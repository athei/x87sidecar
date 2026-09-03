/*
 * test_x87_signal_storm.c — x87 code must survive asynchronous signals.
 *
 * The main thread runs an x87 sequence in a loop while a second thread sends
 * it SIGUSR1 once per completed iteration.  Every iteration is compared
 * bit-for-bit (value, TOP and the C0/C2/C3 condition bits) against what the
 * same code produced before the storm started.  A translation is only correct
 * if a signal can land anywhere inside it: the runtime has to recover a
 * precise guest state at the interrupt point, run the guest handler and
 * resume, and it does that by decoding and stepping the translated ARM code.
 * An instruction the runtime cannot decode aborts the process on some Rosetta
 * builds and silently skips part of the guest instruction on others.
 *
 * The first sequence is the 2^x chain from issue #23, which lost the effect of
 * `fld1; faddp` in one execution out of thousands in a live game.  The rest is
 * one case per x87 opcode the sidecar translates, so the whole emitter surface
 * is exercised.
 *
 * argv[1] selects the storm variant (default "all"):
 *   none    no storm (control)
 *   plain   SIGUSR1 storm, handler only counts
 *   modctx  SIGUSR1 storm, handler rewrites the ucontext with its own content
 *   jit     no signals; the second thread maps, runs and unmaps executable
 *           pages in a loop (translation invalidation)
 * argv[2] is the time budget per case in seconds.  STORM_CASE=<name> runs one
 * case only.  Under the chain cases run in every variant; the per-opcode
 * cases run in "plain" only.
 */
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

static int failures = 0;
static double g_budget_chain = 0.15;
static double g_budget_op = 0.03;

/* ── signal machinery ─────────────────────────────────────────────────── */

enum variant { V_NONE, V_PLAIN, V_MODCTX, V_JIT };

static volatile sig_atomic_t g_variant = V_NONE;
static volatile sig_atomic_t g_storm_run = 0;
static volatile sig_atomic_t g_signals = 0;
/* Iterations completed by the main thread; the signaller paces itself on it
 * so that every iteration is hit about once instead of the victim starving in
 * signal delivery. */
static volatile unsigned long g_iters = 0;

static void handler(int sig, siginfo_t* si, void* ctx) {
    (void)sig;
    (void)si;
    g_signals++;
    if (g_variant == V_MODCTX && ctx != NULL) {
        ucontext_t* uc = (ucontext_t*)ctx;
        /* Same content written back.  Bit 1 of rflags reads as 1 on every
         * x86, so setting it changes nothing the guest can observe. */
        uc->uc_mcontext->__ss.__rflags |= 0x2;
        if (uc->uc_mcsize >= sizeof(*uc->uc_mcontext)) {
            _STRUCT_X86_FLOAT_STATE64 fs;
            memcpy(&fs, &uc->uc_mcontext->__fs, sizeof(fs));
            memcpy(&uc->uc_mcontext->__fs, &fs, sizeof(fs));
        }
    }
}

static pthread_t g_main_thread;

static void* storm_thread(void* arg) {
    (void)arg;
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGUSR1);
    pthread_sigmask(SIG_BLOCK, &set, NULL);
    unsigned long last = g_iters;
    while (g_storm_run) {
        if (g_iters != last) {
            last = g_iters;
            pthread_kill(g_main_thread, SIGUSR1);
        } else {
            __asm__ volatile("pause");
        }
    }
    return NULL;
}

static volatile int g_jit_calls = 0;
static volatile int g_jit_unavailable = 0;

static void* jit_thread(void* arg) {
    (void)arg;
    const size_t page = 4096;
    while (g_storm_run) {
        void* p = mmap(NULL, page, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);
        if (p == MAP_FAILED) {
            g_jit_unavailable = 1;
            break;
        }
        unsigned char* c = (unsigned char*)p;
        uint32_t imm = (uint32_t)g_jit_calls;
        c[0] = 0xB8; /* mov eax, imm32 */
        memcpy(c + 1, &imm, 4);
        c[5] = 0xC3; /* ret */
        if (mprotect(p, page, PROT_READ | PROT_EXEC) != 0) {
            munmap(p, page);
            g_jit_unavailable = 1;
            break;
        }
        uint32_t (*fn)(void) = (uint32_t (*)(void))p;
        if (fn() != imm) {
            g_jit_unavailable = 2;
        }
        munmap(p, page);
        g_jit_calls++;
    }
    return NULL;
}

/* ── the x87 sequences ────────────────────────────────────────────────── */

/* Input block, rax points at it.  Offsets are what the asm bodies use. */
struct input {
    double d;          /* +0   */
    float f;           /* +8   */
    float pad_f;       /* +12  */
    int32_t i32;       /* +16  */
    int32_t pad_i;     /* +20  */
    int16_t i16;       /* +24  */
    int16_t pad_s[3];  /* +26  */
    int64_t i64;       /* +32  */
    long double f80;   /* +40, 16 bytes */
    unsigned char bcd[10]; /* +56 */
    unsigned char pad_b[6];
    uint16_t cw;       /* +72  */
    uint16_t pad_c[3];
    double d2;         /* +80  */
    double small;      /* +88  */
    unsigned char scratch[1024] __attribute__((aligned(16))); /* +96 */
};

struct result {
    double value;
    uint16_t sw; /* fnstsw with the sequence's one remaining value on the stack */
};

static uint64_t as_u64(double d) {
    uint64_t u;
    memcpy(&u, &d, 8);
    return u;
}

/* Fixed registers so the Intel-syntax block needs no operand substitution:
 * rax = &input, rdx = &value, rcx = &sw.  Every body leaves exactly one value
 * on the x87 stack; the macro records the status word and pops it. */
#define X87_SEQ(name, body)                                                          \
    static void name(struct input* in, struct result* r) {                           \
        __asm__ volatile(".intel_syntax noprefix\n\t" body                           \
                         "fnstsw word ptr [rcx]\n\t"                                 \
                         "fstp qword ptr [rdx]\n\t"                                  \
                         ".att_syntax prefix\n\t"                                    \
                         :                                                           \
                         : "a"(in), "d"(&r->value), "c"(&r->sw)                      \
                         : "memory", "cc", "st", "st(1)", "st(2)", "st(3)", "st(4)", \
                           "st(5)", "st(6)", "st(7)");                               \
    }

#define D "qword ptr [rax]"
#define F "dword ptr [rax+8]"
#define I32 "dword ptr [rax+16]"
#define I16 "word ptr [rax+24]"
#define I64 "qword ptr [rax+32]"
#define F80 "tbyte ptr [rax+40]"
#define BCD "tbyte ptr [rax+56]"
#define CW "word ptr [rax+72]"
#define D2 "qword ptr [rax+80]"
#define SMALL "qword ptr [rax+88]"
#define S32 "dword ptr [rax+96]"
#define S16 "word ptr [rax+96]"
#define S64 "qword ptr [rax+96]"
#define S80 "tbyte ptr [rax+96]"
#define SMEM "[rax+96]"

/* Issue #23's chain: 2^x. */
X87_SEQ(seq_chain,
        "fld " D "\n\t"
        "fld st(0)\n\t"
        "frndint\n\t"
        "fsubr st(1), st\n\t"
        "fxch\n\t"
        "fchs\n\t"
        "f2xm1\n\t"
        "fld1\n\t"
        "faddp\n\t"
        "fscale\n\t"
        "fstp st(1)\n\t")

/* Loads and constants. */
X87_SEQ(seq_fld1, "fld1\n\t")
X87_SEQ(seq_fldz, "fldz\n\t")
X87_SEQ(seq_fldl2e, "fldl2e\n\t")
X87_SEQ(seq_fldl2t, "fldl2t\n\t")
X87_SEQ(seq_fldlg2, "fldlg2\n\t")
X87_SEQ(seq_fldln2, "fldln2\n\t")
X87_SEQ(seq_fldpi, "fldpi\n\t")
X87_SEQ(seq_fld_m32, "fld " F "\n\t")
X87_SEQ(seq_fld_m64, "fld " D "\n\t")
X87_SEQ(seq_fld_m80, "fld " F80 "\n\t")
X87_SEQ(seq_fld_sti, "fld " D "\n\tfld st(0)\n\tfaddp\n\t")
X87_SEQ(seq_fild16, "fild " I16 "\n\t")
X87_SEQ(seq_fild32, "fild " I32 "\n\t")
X87_SEQ(seq_fild64, "fild " I64 "\n\t")
X87_SEQ(seq_fbld, "fbld " BCD "\n\t")

/* Stores. */
X87_SEQ(seq_fst_m32, "fld " D "\n\tfst " S32 "\n\tfadd " S32 "\n\t")
X87_SEQ(seq_fst_m64, "fld " D "\n\tfst " S64 "\n\tfadd " S64 "\n\t")
X87_SEQ(seq_fstp_m32, "fld " D "\n\tfstp " S32 "\n\tfld " S32 "\n\t")
X87_SEQ(seq_fstp_m64, "fld " D "\n\tfstp " S64 "\n\tfld " S64 "\n\t")
X87_SEQ(seq_fstp_m80, "fld " D "\n\tfstp " S80 "\n\tfld " S80 "\n\t")
X87_SEQ(seq_fst_sti, "fld " D "\n\tfld " D2 "\n\tfst st(1)\n\tfaddp\n\t")
X87_SEQ(seq_fstp_sti, "fld " D "\n\tfld " D2 "\n\tfstp st(1)\n\t")
X87_SEQ(seq_fist16, "fld " D "\n\tfist " S16 "\n\tfild " S16 "\n\tfaddp\n\t")
X87_SEQ(seq_fist32, "fld " D "\n\tfist " S32 "\n\tfild " S32 "\n\tfaddp\n\t")
X87_SEQ(seq_fistp16, "fld " D "\n\tfistp " S16 "\n\tfild " S16 "\n\t")
X87_SEQ(seq_fistp32, "fld " D "\n\tfistp " S32 "\n\tfild " S32 "\n\t")
X87_SEQ(seq_fistp64, "fld " D "\n\tfistp " S64 "\n\tfild " S64 "\n\t")
X87_SEQ(seq_fisttp16, "fld " D "\n\tfisttp " S16 "\n\tfild " S16 "\n\t")
X87_SEQ(seq_fisttp32, "fld " D "\n\tfisttp " S32 "\n\tfild " S32 "\n\t")
X87_SEQ(seq_fisttp64, "fld " D "\n\tfisttp " S64 "\n\tfild " S64 "\n\t")
X87_SEQ(seq_fbstp, "fld " D2 "\n\tfbstp " S80 "\n\tfbld " S80 "\n\t")

/* Arithmetic, register forms. */
X87_SEQ(seq_fadd_st, "fld " D "\n\tfld " D2 "\n\tfadd st, st(1)\n\tfstp st(1)\n\t")
X87_SEQ(seq_faddp, "fld " D "\n\tfld " D2 "\n\tfaddp\n\t")
X87_SEQ(seq_fsub_st, "fld " D "\n\tfld " D2 "\n\tfsub st, st(1)\n\tfstp st(1)\n\t")
X87_SEQ(seq_fsubr_st, "fld " D "\n\tfld " D2 "\n\tfsubr st, st(1)\n\tfstp st(1)\n\t")
X87_SEQ(seq_fsubp, "fld " D "\n\tfld " D2 "\n\tfsubp st(1), st\n\t")
X87_SEQ(seq_fsubrp, "fld " D "\n\tfld " D2 "\n\tfsubrp st(1), st\n\t")
X87_SEQ(seq_fmul_st, "fld " D "\n\tfld " D2 "\n\tfmul st, st(1)\n\tfstp st(1)\n\t")
X87_SEQ(seq_fmulp, "fld " D "\n\tfld " D2 "\n\tfmulp\n\t")
X87_SEQ(seq_fdiv_st, "fld " D "\n\tfld " D2 "\n\tfdiv st, st(1)\n\tfstp st(1)\n\t")
X87_SEQ(seq_fdivr_st, "fld " D "\n\tfld " D2 "\n\tfdivr st, st(1)\n\tfstp st(1)\n\t")
X87_SEQ(seq_fdivp, "fld " D "\n\tfld " D2 "\n\tfdivp st(1), st\n\t")
X87_SEQ(seq_fdivrp, "fld " D "\n\tfld " D2 "\n\tfdivrp st(1), st\n\t")

/* Arithmetic, memory forms. */
X87_SEQ(seq_fadd_m32, "fld " D "\n\tfadd " F "\n\t")
X87_SEQ(seq_fadd_m64, "fld " D "\n\tfadd " D2 "\n\t")
X87_SEQ(seq_fsub_m64, "fld " D "\n\tfsub " D2 "\n\t")
X87_SEQ(seq_fsubr_m64, "fld " D "\n\tfsubr " D2 "\n\t")
X87_SEQ(seq_fmul_m32, "fld " D "\n\tfmul " F "\n\t")
X87_SEQ(seq_fmul_m64, "fld " D "\n\tfmul " D2 "\n\t")
X87_SEQ(seq_fdiv_m64, "fld " D "\n\tfdiv " D2 "\n\t")
X87_SEQ(seq_fdivr_m64, "fld " D "\n\tfdivr " D2 "\n\t")
X87_SEQ(seq_fiadd16, "fld " D "\n\tfiadd " I16 "\n\t")
X87_SEQ(seq_fiadd32, "fld " D "\n\tfiadd " I32 "\n\t")
X87_SEQ(seq_fisub32, "fld " D "\n\tfisub " I32 "\n\t")
X87_SEQ(seq_fisubr32, "fld " D "\n\tfisubr " I32 "\n\t")
X87_SEQ(seq_fimul32, "fld " D "\n\tfimul " I32 "\n\t")
X87_SEQ(seq_fidiv32, "fld " D "\n\tfidiv " I32 "\n\t")
X87_SEQ(seq_fidivr32, "fld " D "\n\tfidivr " I32 "\n\t")

/* Unary and transcendental. */
X87_SEQ(seq_fabs, "fld " D "\n\tfabs\n\t")
X87_SEQ(seq_fchs, "fld " D "\n\tfchs\n\t")
X87_SEQ(seq_fsqrt, "fld " D "\n\tfsqrt\n\t")
X87_SEQ(seq_frndint, "fld " D "\n\tfrndint\n\t")
X87_SEQ(seq_fxtract, "fld " D "\n\tfxtract\n\tfaddp\n\t")
X87_SEQ(seq_fscale, "fld " D2 "\n\tfld " D "\n\tfscale\n\tfstp st(1)\n\t")
X87_SEQ(seq_fprem, "fld " D2 "\n\tfld " D "\n\tfprem\n\tfstp st(1)\n\t")
X87_SEQ(seq_fprem1, "fld " D2 "\n\tfld " D "\n\tfprem1\n\tfstp st(1)\n\t")
X87_SEQ(seq_f2xm1, "fld " D "\n\tf2xm1\n\t")
X87_SEQ(seq_fyl2x, "fld " D2 "\n\tfld " D "\n\tfyl2x\n\t")
X87_SEQ(seq_fyl2xp1, "fld " D2 "\n\tfld " SMALL "\n\tfyl2xp1\n\t")
X87_SEQ(seq_fsin, "fld " D "\n\tfsin\n\t")
X87_SEQ(seq_fcos, "fld " D "\n\tfcos\n\t")
X87_SEQ(seq_fsincos, "fld " D "\n\tfsincos\n\tfaddp\n\t")
X87_SEQ(seq_fptan, "fld " D "\n\tfptan\n\tfaddp\n\t")
X87_SEQ(seq_fpatan, "fld " D2 "\n\tfld " D "\n\tfpatan\n\t")

/* Compares (condition codes land in the recorded status word). */
X87_SEQ(seq_fcom_m64, "fld " D "\n\tfcom " D2 "\n\t")
X87_SEQ(seq_fcom_m32, "fld " D "\n\tfcom " F "\n\t")
X87_SEQ(seq_fcom_st, "fld " D "\n\tfld " D2 "\n\tfcom st(1)\n\tfstp st(1)\n\t")
X87_SEQ(seq_fcomp, "fld " D "\n\tfld " D2 "\n\tfcomp st(1)\n\t")
X87_SEQ(seq_fcompp, "fld " D "\n\tfld " D2 "\n\tfcompp\n\tfld " D "\n\t")
X87_SEQ(seq_fucom, "fld " D "\n\tfld " D2 "\n\tfucom st(1)\n\tfstp st(1)\n\t")
X87_SEQ(seq_fucomp, "fld " D "\n\tfld " D2 "\n\tfucomp st(1)\n\t")
X87_SEQ(seq_fucompp, "fld " D "\n\tfld " D2 "\n\tfucompp\n\tfld " D "\n\t")
X87_SEQ(seq_fcomi, "fld " D "\n\tfld " D2 "\n\tfcomi st, st(1)\n\tfstp st(1)\n\t")
X87_SEQ(seq_fcomip, "fld " D "\n\tfld " D2 "\n\tfcomip st, st(1)\n\t")
X87_SEQ(seq_fucomi, "fld " D "\n\tfld " D2 "\n\tfucomi st, st(1)\n\tfstp st(1)\n\t")
X87_SEQ(seq_fucomip, "fld " D "\n\tfld " D2 "\n\tfucomip st, st(1)\n\t")
X87_SEQ(seq_ficom16, "fld " D "\n\tficom " I16 "\n\t")
X87_SEQ(seq_ficom32, "fld " D "\n\tficom " I32 "\n\t")
X87_SEQ(seq_ficomp32, "fld " D "\n\tficomp " I32 "\n\tfld " D "\n\t")
X87_SEQ(seq_ftst, "fld " D "\n\tftst\n\t")
X87_SEQ(seq_fxam, "fld " D "\n\tfxam\n\t")
X87_SEQ(seq_fcmovb, "fld " D "\n\tfld " D2 "\n\tfcomi st, st(1)\n\tfcmovb st, st(1)\n\tfstp st(1)\n\t")
X87_SEQ(seq_fcmove, "fld " D "\n\tfld " D2 "\n\tfcomi st, st(1)\n\tfcmove st, st(1)\n\tfstp st(1)\n\t")
X87_SEQ(seq_fcmovbe, "fld " D "\n\tfld " D2 "\n\tfcomi st, st(1)\n\tfcmovbe st, st(1)\n\tfstp st(1)\n\t")
X87_SEQ(seq_fcmovu, "fld " D "\n\tfld " D2 "\n\tfcomi st, st(1)\n\tfcmovu st, st(1)\n\tfstp st(1)\n\t")
X87_SEQ(seq_fcmovnb, "fld " D "\n\tfld " D2 "\n\tfcomi st, st(1)\n\tfcmovnb st, st(1)\n\tfstp st(1)\n\t")
X87_SEQ(seq_fcmovne, "fld " D "\n\tfld " D2 "\n\tfcomi st, st(1)\n\tfcmovne st, st(1)\n\tfstp st(1)\n\t")
X87_SEQ(seq_fcmovnbe, "fld " D "\n\tfld " D2 "\n\tfcomi st, st(1)\n\tfcmovnbe st, st(1)\n\tfstp st(1)\n\t")
X87_SEQ(seq_fcmovnu, "fld " D "\n\tfld " D2 "\n\tfcomi st, st(1)\n\tfcmovnu st, st(1)\n\tfstp st(1)\n\t")

/* Stack and control. */
X87_SEQ(seq_fxch, "fld " D "\n\tfld " D2 "\n\tfxch\n\tfstp st(1)\n\t")
X87_SEQ(seq_fxch2, "fld " D "\n\tfld " D2 "\n\tfld1\n\tfxch st(2)\n\tfaddp\n\tfaddp\n\t")
X87_SEQ(seq_ffree, "fld " D "\n\tfld " D2 "\n\tffree st(1)\n\tfstp st(1)\n\t")
X87_SEQ(seq_fincdec, "fld " D "\n\tfdecstp\n\tfincstp\n\t")
X87_SEQ(seq_fldcw, "fldcw " CW "\n\tfld " D "\n\t")
X87_SEQ(seq_fnstcw, "fld " D "\n\tfnstcw " S16 "\n\tfldcw " S16 "\n\t")
X87_SEQ(seq_fnstsw, "fld " D "\n\tfnstsw " S16 "\n\t")
X87_SEQ(seq_fnclex, "fld " D "\n\tfnclex\n\t")
X87_SEQ(seq_fninit, "fninit\n\tfld " D "\n\t")
X87_SEQ(seq_fnstenv, "fld " D "\n\tfnstenv " SMEM "\n\tfldenv " SMEM "\n\t")
X87_SEQ(seq_fnsave, "fld " D "\n\tfnsave " SMEM "\n\tfrstor " SMEM "\n\t")
X87_SEQ(seq_fxsave, "fld " D "\n\tfxsave " SMEM "\n\tfxrstor " SMEM "\n\t")
X87_SEQ(seq_fnop, "fld " D "\n\tfnop\n\t")
X87_SEQ(seq_fdisi_feni, "fld " D "\n\t.byte 0xdb, 0xe1\n\t.byte 0xdb, 0xe0\n\t") /* fndisi; fneni */

/* A long run: many consecutive x87 ops, which the IR path translates as one
 * unit.  A signal inside it makes the runtime step the whole rest of the run. */
X87_SEQ(seq_longrun,
        "fld " D "\n\tfmul " D2 "\n\tfadd " D "\n\tfsub " SMALL "\n\tfmul " D2 "\n\t"
        "fadd " D "\n\tfdiv " D2 "\n\tfadd " D "\n\tfmul " D2 "\n\tfsub " D "\n\t"
        "fadd " SMALL "\n\tfld " D2 "\n\tfmulp\n\tfld1\n\tfaddp\n\tfld " SMALL "\n\t"
        "fxch\n\tfsubp st(1), st\n\tfchs\n\tfabs\n\t")

typedef void (*seq_fn)(struct input*, struct result*);

struct op_case {
    const char* name;
    seq_fn fn;
};

static const struct op_case k_op_cases[] = {
    {"fld1", seq_fld1},           {"fldz", seq_fldz},           {"fldl2e", seq_fldl2e},
    {"fldl2t", seq_fldl2t},       {"fldlg2", seq_fldlg2},       {"fldln2", seq_fldln2},
    {"fldpi", seq_fldpi},         {"fld_m32", seq_fld_m32},     {"fld_m64", seq_fld_m64},
    {"fld_m80", seq_fld_m80},     {"fld_sti", seq_fld_sti},     {"fild16", seq_fild16},
    {"fild32", seq_fild32},       {"fild64", seq_fild64},       {"fbld", seq_fbld},
    {"fst_m32", seq_fst_m32},     {"fst_m64", seq_fst_m64},     {"fstp_m32", seq_fstp_m32},
    {"fstp_m64", seq_fstp_m64},   {"fstp_m80", seq_fstp_m80},   {"fst_sti", seq_fst_sti},
    {"fstp_sti", seq_fstp_sti},   {"fist16", seq_fist16},       {"fist32", seq_fist32},
    {"fistp16", seq_fistp16},     {"fistp32", seq_fistp32},     {"fistp64", seq_fistp64},
    {"fisttp16", seq_fisttp16},   {"fisttp32", seq_fisttp32},   {"fisttp64", seq_fisttp64},
    {"fbstp", seq_fbstp},         {"fadd_st", seq_fadd_st},     {"faddp", seq_faddp},
    {"fsub_st", seq_fsub_st},     {"fsubr_st", seq_fsubr_st},   {"fsubp", seq_fsubp},
    {"fsubrp", seq_fsubrp},       {"fmul_st", seq_fmul_st},     {"fmulp", seq_fmulp},
    {"fdiv_st", seq_fdiv_st},     {"fdivr_st", seq_fdivr_st},   {"fdivp", seq_fdivp},
    {"fdivrp", seq_fdivrp},       {"fadd_m32", seq_fadd_m32},   {"fadd_m64", seq_fadd_m64},
    {"fsub_m64", seq_fsub_m64},   {"fsubr_m64", seq_fsubr_m64}, {"fmul_m32", seq_fmul_m32},
    {"fmul_m64", seq_fmul_m64},   {"fdiv_m64", seq_fdiv_m64},   {"fdivr_m64", seq_fdivr_m64},
    {"fiadd16", seq_fiadd16},     {"fiadd32", seq_fiadd32},     {"fisub32", seq_fisub32},
    {"fisubr32", seq_fisubr32},   {"fimul32", seq_fimul32},     {"fidiv32", seq_fidiv32},
    {"fidivr32", seq_fidivr32},   {"fabs", seq_fabs},           {"fchs", seq_fchs},
    {"fsqrt", seq_fsqrt},         {"frndint", seq_frndint},     {"fxtract", seq_fxtract},
    {"fscale", seq_fscale},       {"fprem", seq_fprem},         {"fprem1", seq_fprem1},
    {"f2xm1", seq_f2xm1},         {"fyl2x", seq_fyl2x},         {"fyl2xp1", seq_fyl2xp1},
    {"fsin", seq_fsin},           {"fcos", seq_fcos},           {"fsincos", seq_fsincos},
    {"fptan", seq_fptan},         {"fpatan", seq_fpatan},       {"fcom_m64", seq_fcom_m64},
    {"fcom_m32", seq_fcom_m32},   {"fcom_st", seq_fcom_st},     {"fcomp", seq_fcomp},
    {"fcompp", seq_fcompp},       {"fucom", seq_fucom},         {"fucomp", seq_fucomp},
    {"fucompp", seq_fucompp},     {"fcomi", seq_fcomi},         {"fcomip", seq_fcomip},
    {"fucomi", seq_fucomi},       {"fucomip", seq_fucomip},     {"ficom16", seq_ficom16},
    {"ficom32", seq_ficom32},     {"ficomp32", seq_ficomp32},   {"ftst", seq_ftst},
    {"fxam", seq_fxam},           {"fcmovb", seq_fcmovb},       {"fcmove", seq_fcmove},
    {"fcmovbe", seq_fcmovbe},     {"fcmovu", seq_fcmovu},       {"fcmovnb", seq_fcmovnb},
    {"fcmovne", seq_fcmovne},     {"fcmovnbe", seq_fcmovnbe},   {"fcmovnu", seq_fcmovnu},
    {"fxch", seq_fxch},           {"fxch2", seq_fxch2},         {"ffree", seq_ffree},
    {"fincdec", seq_fincdec},     {"fldcw", seq_fldcw},         {"fnstcw", seq_fnstcw},
    {"fnstsw", seq_fnstsw},       {"fnclex", seq_fnclex},       {"fninit", seq_fninit},
    {"fnstenv", seq_fnstenv},     {"fnsave", seq_fnsave},       {"fxsave", seq_fxsave},
    {"fnop", seq_fnop},           {"fdisi_feni", seq_fdisi_feni}, {"longrun", seq_longrun},
};

/* ── driver ───────────────────────────────────────────────────────────── */

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static const char* variant_name(int v) {
    switch (v) {
        case V_NONE:
            return "none";
        case V_PLAIN:
            return "plain";
        case V_MODCTX:
            return "modctx";
        case V_JIT:
            return "jit";
        default:
            return "?";
    }
}

static struct input g_in;

static void init_input(double x) {
    memset(&g_in, 0, sizeof(g_in));
    g_in.d = x;
    g_in.f = 1.5f;
    g_in.i32 = 7;
    g_in.i16 = -3;
    g_in.i64 = 123456789;
    g_in.f80 = 2.5L;
    g_in.bcd[0] = 0x34; /* 1234 */
    g_in.bcd[1] = 0x12;
    g_in.cw = 0x037F;
    g_in.d2 = 3.0;
    g_in.small = 0.125;
}

/* Bits of the status word that must be stable: TOP and C0/C2/C3.  C1 is the
 * rounding/stack-fault indicator and is not compared. */
#define SW_MASK 0x7D00

static void run_case(const char* name, seq_fn fn, double x, int variant, double budget_s,
                     double expect_hint) {
    const char* only = getenv("STORM_CASE");
    if (only != NULL && strcmp(only, name) != 0) {
        return;
    }

    struct result ref;
    struct result got;
    init_input(x);
    memset(&ref, 0, sizeof(ref));
    fn(&g_in, &ref);

    if (!isnan(expect_hint) && fabs(ref.value - expect_hint) > 1e-9 * fabs(expect_hint) + 1e-12) {
        printf("FAIL  %s x=%.17g variant=%s  reference wrong\n", name, x, variant_name(variant));
        printf("      ref=%.17g (0x%016llx)  hint=%.17g\n", ref.value,
               (unsigned long long)as_u64(ref.value), expect_hint);
        failures++;
        return;
    }

    g_variant = variant;
    g_signals = 0;
    g_jit_calls = 0;
    g_jit_unavailable = 0;
    g_storm_run = 1;

    pthread_t th;
    int have_thread = 0;
    if (variant == V_PLAIN || variant == V_MODCTX) {
        have_thread = pthread_create(&th, NULL, storm_thread, NULL) == 0;
    } else if (variant == V_JIT) {
        have_thread = pthread_create(&th, NULL, jit_thread, NULL) == 0;
    }

    unsigned long iters = 0;
    unsigned long mismatches = 0;
    unsigned long first_iter = 0;
    struct result first;
    memset(&first, 0, sizeof(first));

    const double deadline = now_s() + budget_s;
    for (;;) {
        for (int k = 0; k < 64; k++) {
            init_input(x); /* stores in the sequence may have changed scratch */
            memset(&got, 0, sizeof(got));
            fn(&g_in, &got);
            iters++;
            g_iters = iters;
            if (as_u64(got.value) != as_u64(ref.value) || (got.sw & SW_MASK) != (ref.sw & SW_MASK)) {
                if (mismatches == 0) {
                    first = got;
                    first_iter = iters;
                }
                mismatches++;
            }
        }
        if (now_s() >= deadline) {
            break;
        }
    }

    g_storm_run = 0;
    if (have_thread) {
        pthread_join(th, NULL);
    }

    if (mismatches == 0) {
        printf("PASS  %s x=%.17g variant=%s  iters=%lu signals=%d jit=%d%s\n", name, x,
               variant_name(variant), iters, (int)g_signals, g_jit_calls,
               g_jit_unavailable ? " (jit page unavailable)" : "");
    } else {
        printf("FAIL  %s x=%.17g variant=%s\n", name, x, variant_name(variant));
        printf("      mismatches=%lu of %lu, first at iteration %lu, signals=%d\n", mismatches, iters,
               first_iter, (int)g_signals);
        printf("      got=%.17g (0x%016llx) sw=%04x\n", first.value,
               (unsigned long long)as_u64(first.value), first.sw & SW_MASK);
        printf("      exp=%.17g (0x%016llx) sw=%04x\n", ref.value,
               (unsigned long long)as_u64(ref.value), ref.sw & SW_MASK);
        failures++;
    }
}

int main(int argc, char** argv) {
    const char* which = argc > 1 ? argv[1] : "all";
    if (argc > 2) {
        g_budget_chain = atof(argv[2]);
        g_budget_op = g_budget_chain / 5.0;
    }

    g_main_thread = pthread_self();

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = handler;
    sa.sa_flags = SA_SIGINFO | SA_RESTART;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGUSR1, &sa, NULL) != 0) {
        printf("FAIL  sigaction\n");
        return 1;
    }

    int variants[4];
    int nvariants = 0;
    if (strcmp(which, "all") == 0) {
        variants[nvariants++] = V_NONE;
        variants[nvariants++] = V_PLAIN;
        variants[nvariants++] = V_MODCTX;
        variants[nvariants++] = V_JIT;
    } else if (strcmp(which, "none") == 0) {
        variants[nvariants++] = V_NONE;
    } else if (strcmp(which, "plain") == 0) {
        variants[nvariants++] = V_PLAIN;
    } else if (strcmp(which, "modctx") == 0) {
        variants[nvariants++] = V_MODCTX;
    } else if (strcmp(which, "jit") == 0) {
        variants[nvariants++] = V_JIT;
    } else {
        printf("FAIL  unknown variant %s\n", which);
        return 1;
    }

    for (int i = 0; i < nvariants; i++) {
        const int v = variants[i];
        /* The reporter's value: n = 0, so a lost `+1` reads 2^x - 1. */
        run_case("chain", seq_chain, -0.005, v, g_budget_chain, pow(2.0, -0.005));
        /* n != 0 exercises fscale for real. */
        run_case("chain", seq_chain, 2.3, v, g_budget_chain, pow(2.0, 2.3));
        run_case("chain", seq_chain, -3.7, v, g_budget_chain, pow(2.0, -3.7));
        if (v == V_PLAIN) {
            for (size_t k = 0; k < sizeof(k_op_cases) / sizeof(k_op_cases[0]); k++) {
                run_case(k_op_cases[k].name, k_op_cases[k].fn, 0.75, v, g_budget_op, NAN);
            }
        }
    }

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
