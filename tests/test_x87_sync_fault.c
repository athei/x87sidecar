/*
 * An x87 memory access that faults must reach the program's SIGSEGV/SIGBUS
 * handler with a precise context, as under stock Rosetta: rip at the faulting
 * instruction, and the x87 stack, TOP, status word and control word as they
 * were before it.  Windows programs under wine rely on this through
 * structured exception handling (guard pages, IsBadReadPtr, deliberate
 * access violations).
 *
 * Every case runs one x87 instruction against a PROT_NONE page: alone in
 * its translation reply, at the start of a longer x87 run (also one joined
 * across a mov by bridging), as the first half of a two-instruction run that
 * a fusion takes, in a run of integer loads and stores only, and straddling
 * the page boundary.  The handler checks the context against a reference
 * taken just before the instruction, then either makes the page accessible
 * and returns, so the instruction runs again, or moves rip past the
 * instruction.  The final x87 state and memory are compared with an
 * unfaulted run and with a run that leaves the instruction out.
 *
 * The sidecar answers a run of floating-point x87 instructions with one
 * reply, and the runtime can only roll a fault back to the start of a
 * reply, so a fault in an instruction that is not the first of its reply
 * still aborts the process (docs/internals.md, "Memory faults inside
 * emitted code").  The store of a fused fild m64 / fistp m64 pair is
 * rolled back to the fild instead.  Those cases are in a second group that
 * runs only with SYNC_FAULT_IN_RUN=1; under stock Rosetta they pass.
 *
 * SYNC_FAULT_CASE=<name> runs one case of either group; SYNC_FAULT_LIST=1
 * prints every name with its group.
 */
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

/* Layout shared with the assembly below. */
struct io {
    double v[4];          /* 0x00 */
    uint16_t cw;          /* 0x20 */
    uint16_t pad0;        /* 0x22 */
    int32_t i32src;       /* 0x24 */
    char* target;         /* 0x28 */
    char* scratch;        /* 0x30 */
    int64_t i64src;       /* 0x38 */
    uint8_t m80src[16];   /* 0x40 */
    uint8_t save[112];    /* 0x50, fnsave image */
    int16_t i16src;       /* 0xc0 */
};
_Static_assert(__builtin_offsetof(struct io, save) == 0x50, "io layout");
_Static_assert(__builtin_offsetof(struct io, i16src) == 0xc0, "io layout");

/* Each case is three functions: the case itself, a reference that stops
 * where the instruction would run, and the case without the instruction. */
#define SETUP                         \
    "fninit\n"                        \
    "fldcw 0x20(%rdi)\n"              \
    "mov 0x28(%rdi), %rsi\n"          \
    "mov 0x30(%rdi), %rdx\n"          \
    "fldl 0x00(%rdi)\n"               \
    "fldl 0x08(%rdi)\n"               \
    "lfence\n"
#define FINISH            \
    "\nlfence\n"          \
    "fnsave 0x50(%rdi)\n" \
    "ret\n"

#define DEFINE_CASE(group, name, init, off, pre, insn, post)                                  \
    __asm__(".text\n.p2align 4\n.globl _" #name "\n_" #name ":\n" SETUP pre                   \
            "\n.globl _" #name "_fault\n_" #name "_fault:\n" insn "\n.globl _" #name            \
            "_after\n_" #name "_after:\n" post FINISH ".p2align 4\n.globl _" #name              \
            "_ref\n_" #name "_ref:\n" SETUP pre FINISH ".p2align 4\n.globl _" #name "_skip\n_" \
            #name "_skip:\n" SETUP pre "\n" post FINISH);                                      \
    extern void name(struct io*);                                                             \
    extern void name##_ref(struct io*);                                                       \
    extern void name##_skip(struct io*);                                                      \
    extern char name##_fault[], name##_after[];

enum init { M_NONE, M_F32, M_F64, M_F80, M_I16, M_I32, M_I64, M_BCD, M_CW, M_ENV, M_SAVE };

/* Groups: PRECISE must pass everywhere; IN_RUN is not precise under the
 * sidecar yet and runs only on request. */
enum group { PRECISE, IN_RUN };

/* The operand normally sits well inside the protected page; CROSS puts it
 * 4 bytes before it, so an access of more than 4 bytes straddles the
 * boundary. */
#define IN 0x100
#define CROSS (-4)

/* Run shapes.  RUN_PRE/RUN_POST make a run of five or more x87
 * instructions around the faulting one, which the IR pipeline lowers as a
 * whole.  BRIDGE_POST is a run that bridging joins across a mov. */
#define RUN_PRE "fldl 0x10(%rdi)\nfmul %st(1), %st\n"
#define RUN_POST "fldl 0x18(%rdi)\nfmulp\nfadd %st(1), %st\n"
#define BRIDGE_POST "fmull 0x10(%rdi)\nmov 0x38(%rdi), %rcx\nfaddl 0x18(%rdi)\nfstpl (%rdx)\n"
/* Integer loads and stores only: a native transport run.  TR_PRE ends in
 * an m16 load so that an fistpll after it is not a fused copy pair. */
#define TR_PRE "fildll 0x38(%rdi)\nfilds 0xc0(%rdi)\n"
#define TR_POST "fistpl 8(%rdx)\nfildl 0x24(%rdi)\nfistpll 16(%rdx)\n"

#define CASES(X)                                                                                \
    /* alone in a reply */                                                                      \
    X(PRECISE, s_fld32, M_F32, IN, "", "flds (%rsi)", "")                                       \
    X(PRECISE, s_fld64, M_F64, IN, "", "fldl (%rsi)", "")                                       \
    X(PRECISE, s_fld80, M_F80, IN, "", "fldt (%rsi)", "")                                       \
    X(PRECISE, s_fst32, M_NONE, IN, "", "fsts (%rsi)", "")                                      \
    X(PRECISE, s_fst64, M_NONE, IN, "", "fstl (%rsi)", "")                                      \
    X(PRECISE, s_fstp32, M_NONE, IN, "", "fstps (%rsi)", "")                                    \
    X(PRECISE, s_fstp64, M_NONE, IN, "", "fstpl (%rsi)", "")                                    \
    X(PRECISE, s_fstp80, M_NONE, IN, "", "fstpt (%rsi)", "")                                    \
    X(PRECISE, s_fild16, M_I16, IN, "", "filds (%rsi)", "")                                     \
    X(PRECISE, s_fild32, M_I32, IN, "", "fildl (%rsi)", "")                                     \
    X(PRECISE, s_fild64, M_I64, IN, "", "fildll (%rsi)", "")                                    \
    X(PRECISE, s_fist16, M_NONE, IN, "", "fists (%rsi)", "")                                    \
    X(PRECISE, s_fist32, M_NONE, IN, "", "fistl (%rsi)", "")                                    \
    X(PRECISE, s_fistp16, M_NONE, IN, "", "fistps (%rsi)", "")                                  \
    X(PRECISE, s_fistp32, M_NONE, IN, "", "fistpl (%rsi)", "")                                  \
    X(PRECISE, s_fistp64, M_NONE, IN, "", "fistpll (%rsi)", "")                                 \
    X(PRECISE, s_fisttp32, M_NONE, IN, "", "fisttpl (%rsi)", "")                                \
    X(PRECISE, s_fbld, M_BCD, IN, "", "fbld (%rsi)", "")                                        \
    X(PRECISE, s_fbstp, M_NONE, IN, "", "fbstp (%rsi)", "")                                     \
    X(PRECISE, s_fldcw, M_CW, IN, "", "fldcw (%rsi)", "")                                       \
    X(PRECISE, s_fnstcw, M_NONE, IN, "", "fnstcw (%rsi)", "")                                   \
    X(PRECISE, s_fnstsw, M_NONE, IN, "", "fnstsw (%rsi)", "")                                   \
    X(PRECISE, s_fnsave, M_NONE, IN, "", "fnsave (%rsi)", "")                                   \
    X(PRECISE, s_frstor, M_SAVE, IN, "", "frstor (%rsi)", "")                                   \
    X(PRECISE, s_fnstenv, M_NONE, IN, "", "fnstenv (%rsi)", "")                                 \
    X(PRECISE, s_fldenv, M_ENV, IN, "", "fldenv (%rsi)", "")                                    \
    X(PRECISE, s_fadd64, M_F64, IN, "", "faddl (%rsi)", "")                                     \
    X(PRECISE, s_fmul32, M_F32, IN, "", "fmuls (%rsi)", "")                                     \
    X(PRECISE, s_fsubr64, M_F64, IN, "", "fsubrl (%rsi)", "")                                   \
    X(PRECISE, s_fdiv32, M_F32, IN, "", "fdivs (%rsi)", "")                                     \
    X(PRECISE, s_fcom64, M_F64, IN, "", "fcoml (%rsi)", "")                                     \
    X(PRECISE, s_fcomp32, M_F32, IN, "", "fcomps (%rsi)", "")                                   \
    X(PRECISE, s_fiadd32, M_I32, IN, "", "fiaddl (%rsi)", "")                                   \
    X(PRECISE, s_fimul16, M_I16, IN, "", "fimuls (%rsi)", "")                                   \
    X(PRECISE, s_ficom32, M_I32, IN, "", "ficoml (%rsi)", "")                                   \
    /* first instruction of a longer run */                                                     \
    X(PRECISE, r_first_fld64, M_F64, IN, "", "fldl (%rsi)", RUN_POST)                           \
    X(PRECISE, r_first_fld80, M_F80, IN, "", "fldt (%rsi)", RUN_POST)                           \
    X(PRECISE, r_first_fild32, M_I32, IN, "", "fildl (%rsi)", RUN_POST)                         \
    X(PRECISE, r_first_fadd64, M_F64, IN, "", "faddl (%rsi)", RUN_POST)                         \
    X(PRECISE, r_first_fstp64, M_NONE, IN, "", "fstpl (%rsi)", RUN_POST)                        \
    X(PRECISE, r_first_fistp32, M_NONE, IN, "", "fistpl (%rsi)", RUN_POST)                      \
    X(PRECISE, r_first_fnstcw, M_NONE, IN, "", "fnstcw (%rsi)", RUN_POST)                       \
    X(PRECISE, b_first_fld64, M_F64, IN, "", "fldl (%rsi)", BRIDGE_POST)                        \
    X(PRECISE, b_first_fstp64, M_NONE, IN, "", "fstpl (%rsi)", "fldl 0x18(%rdi)\n" BRIDGE_POST) \
    /* first half of a two-instruction run that a fusion takes */                               \
    X(PRECISE, f_fld_fstp_load, M_F64, IN, "", "fldl (%rsi)", "fstpl (%rdx)")                   \
    X(PRECISE, f_m80_load, M_F80, IN, "", "fldt (%rsi)", "fstpt 0x20(%rdx)")                    \
    X(PRECISE, f_arith_fstp_load, M_F64, IN, "", "fmull (%rsi)", "fstpl (%rdx)")                \
    X(PRECISE, f_fstp_fld_store, M_NONE, IN, "", "fstpl (%rsi)", "fldl 0x10(%rdi)")             \
    X(PRECISE, f_fld_arithp, M_F64, IN, "", "fldl (%rsi)", "faddp")                             \
    X(PRECISE, f_fld_fcomp, M_F64, IN, "", "fldl (%rsi)", "fcompl 0x10(%rdi)")                  \
    X(PRECISE, f_fld_arith_fstp_load, M_F64, IN, "", "fldl (%rsi)",                             \
      "fmull 0x10(%rdi)\nfstpl (%rdx)")                                                         \
    /* integer loads and stores only */                                                         \
    X(PRECISE, t_fild16, M_I16, IN, TR_PRE, "filds (%rsi)", "fistps (%rdx)\n" TR_POST)          \
    X(PRECISE, t_fild32, M_I32, IN, TR_PRE, "fildl (%rsi)", "fistpl (%rdx)\n" TR_POST)          \
    X(PRECISE, t_fild64, M_I64, IN, TR_PRE, "fildll (%rsi)", "fistpll (%rdx)\n" TR_POST)        \
    X(PRECISE, t_fist32, M_NONE, IN, TR_PRE, "fistl (%rsi)", TR_POST)                           \
    X(PRECISE, t_fistp16, M_NONE, IN, TR_PRE, "fistps (%rsi)", "fildll 0x38(%rdi)\n" TR_POST)   \
    X(PRECISE, t_fistp64, M_NONE, IN, TR_PRE, "fistpll (%rsi)", "fildll 0x38(%rdi)\n" TR_POST)  \
    X(PRECISE, t_fisttp32, M_NONE, IN, TR_PRE, "fisttpl (%rsi)", "fildll 0x38(%rdi)\n" TR_POST) \
    X(PRECISE, t_fldcw, M_CW, IN, TR_PRE, "fldcw (%rsi)", TR_POST)                              \
    X(PRECISE, t_fnstcw, M_NONE, IN, TR_PRE, "fnstcw (%rsi)", TR_POST)                          \
    X(PRECISE, t_first_fild64, M_I64, IN, "", "fildll (%rsi)", "fistpll (%rdx)\n" TR_POST)      \
    X(PRECISE, t_pair_load, M_I64, IN, "", "fildll (%rsi)", "fistpll (%rdx)")                   \
    X(PRECISE, t_nested_store, M_NONE, IN, "fildll 0x38(%rdi)\nfildll 0x38(%rdi)\n"              \
      "fistpll 8(%rdx)\n", "fistpll (%rsi)", "")                                                \
    /* the access straddles into the protected page */                                          \
    X(PRECISE, x_fld64, M_NONE, CROSS, "", "fldl (%rsi)", "")                                   \
    X(PRECISE, x_fld80, M_NONE, CROSS, "", "fldt (%rsi)", RUN_POST)                             \
    X(PRECISE, x_fstp64, M_NONE, CROSS, "", "fstpl (%rsi)", "")                                 \
    X(PRECISE, x_fistp64, M_NONE, CROSS, TR_PRE, "fistpll (%rsi)", TR_POST)                     \
    X(PRECISE, x_fnsave, M_NONE, CROSS, "", "fnsave (%rsi)", "")                                \
    /* later in a floating-point run: still aborts under the sidecar */                         \
    X(IN_RUN, r_fld32, M_F32, IN, RUN_PRE, "flds (%rsi)", RUN_POST)                             \
    X(IN_RUN, r_fld64, M_F64, IN, RUN_PRE, "fldl (%rsi)", RUN_POST)                             \
    X(IN_RUN, r_fld80, M_F80, IN, RUN_PRE, "fldt (%rsi)", RUN_POST)                             \
    X(IN_RUN, r_fst32, M_NONE, IN, RUN_PRE, "fsts (%rsi)", RUN_POST)                            \
    X(IN_RUN, r_fst64, M_NONE, IN, RUN_PRE, "fstl (%rsi)", RUN_POST)                            \
    X(IN_RUN, r_fstp32, M_NONE, IN, RUN_PRE, "fstps (%rsi)", RUN_POST)                          \
    X(IN_RUN, r_fstp64, M_NONE, IN, RUN_PRE, "fstpl (%rsi)", RUN_POST)                          \
    X(IN_RUN, r_fstp80, M_NONE, IN, RUN_PRE, "fstpt (%rsi)", RUN_POST)                          \
    X(IN_RUN, r_fild16, M_I16, IN, RUN_PRE, "filds (%rsi)", RUN_POST)                           \
    X(IN_RUN, r_fild32, M_I32, IN, RUN_PRE, "fildl (%rsi)", RUN_POST)                           \
    X(IN_RUN, r_fild64, M_I64, IN, RUN_PRE, "fildll (%rsi)", RUN_POST)                          \
    X(IN_RUN, r_fist16, M_NONE, IN, RUN_PRE, "fists (%rsi)", RUN_POST)                          \
    X(IN_RUN, r_fist32, M_NONE, IN, RUN_PRE, "fistl (%rsi)", RUN_POST)                          \
    X(IN_RUN, r_fistp32, M_NONE, IN, RUN_PRE, "fistpl (%rsi)", RUN_POST)                        \
    X(IN_RUN, r_fistp64, M_NONE, IN, RUN_PRE, "fistpll (%rsi)", RUN_POST)                       \
    X(IN_RUN, r_fisttp32, M_NONE, IN, RUN_PRE, "fisttpl (%rsi)", RUN_POST)                      \
    X(IN_RUN, r_fbld, M_BCD, IN, RUN_PRE, "fbld (%rsi)", RUN_POST)                              \
    X(IN_RUN, r_fbstp, M_NONE, IN, RUN_PRE, "fbstp (%rsi)", RUN_POST)                           \
    X(IN_RUN, r_fldcw, M_CW, IN, RUN_PRE, "fldcw (%rsi)", RUN_POST)                             \
    X(IN_RUN, r_fnstcw, M_NONE, IN, RUN_PRE, "fnstcw (%rsi)", RUN_POST)                         \
    X(IN_RUN, r_fnstsw, M_NONE, IN, RUN_PRE, "fnstsw (%rsi)", RUN_POST)                         \
    X(IN_RUN, r_fnsave, M_NONE, IN, RUN_PRE, "fnsave (%rsi)", RUN_POST)                         \
    X(IN_RUN, r_frstor, M_SAVE, IN, RUN_PRE, "frstor (%rsi)", RUN_POST)                         \
    X(IN_RUN, r_fadd64, M_F64, IN, RUN_PRE, "faddl (%rsi)", RUN_POST)                           \
    X(IN_RUN, r_fmul32, M_F32, IN, RUN_PRE, "fmuls (%rsi)", RUN_POST)                           \
    X(IN_RUN, r_fsubr64, M_F64, IN, RUN_PRE, "fsubrl (%rsi)", RUN_POST)                         \
    X(IN_RUN, r_fdiv32, M_F32, IN, RUN_PRE, "fdivs (%rsi)", RUN_POST)                           \
    X(IN_RUN, r_fcom64, M_F64, IN, RUN_PRE, "fcoml (%rsi)", RUN_POST)                           \
    X(IN_RUN, r_fcomp32, M_F32, IN, RUN_PRE, "fcomps (%rsi)", RUN_POST)                         \
    X(IN_RUN, r_fiadd32, M_I32, IN, RUN_PRE, "fiaddl (%rsi)", RUN_POST)                         \
    X(IN_RUN, r_fimul16, M_I16, IN, RUN_PRE, "fimuls (%rsi)", RUN_POST)                         \
    X(IN_RUN, r_ficom32, M_I32, IN, RUN_PRE, "ficoml (%rsi)", RUN_POST)                         \
    X(IN_RUN, r_last_fld64, M_F64, IN, RUN_PRE "fldl 0x18(%rdi)\nfaddp\n", "fldl (%rsi)", "")   \
    X(IN_RUN, r_last_fstp64, M_NONE, IN, RUN_PRE "fldl 0x18(%rdi)\nfaddp\n", "fstpl (%rsi)",    \
      "")                                                                                       \
    X(IN_RUN, b_mid_fld64, M_F64, IN, "fldl 0x10(%rdi)\nmov 0x38(%rdi), %rcx\n", "fldl (%rsi)", \
      "faddp\nfstpl (%rdx)")                                                                    \
    X(IN_RUN, f_fld_fstp_store, M_NONE, IN, "fldl 0x10(%rdi)\n", "fstpl (%rsi)", "")            \
    X(IN_RUN, f_fld_fstp32_store, M_NONE, IN, "fldl 0x18(%rdi)\n", "fstps (%rsi)", "")          \
    X(IN_RUN, f_m80_store, M_NONE, IN, "fldt 0x40(%rdi)\n", "fstpt (%rsi)", "")                 \
    X(IN_RUN, f_arith_fstp_store, M_NONE, IN, "fmull 0x10(%rdi)\n", "fstpl (%rsi)", "")         \
    X(IN_RUN, f_fstp_fld_load, M_F64, IN, "fstpl (%rdx)\n", "fldl (%rsi)", "")                  \
    X(IN_RUN, f_fld_arith_fstp_mid, M_F64, IN, "fldl 0x10(%rdi)\n", "fmull (%rsi)",             \
      "fstpl (%rdx)")                                                                           \
    X(IN_RUN, f_fld_arith_fstp_store, M_NONE, IN, "fldl 0x10(%rdi)\nfmull 0x18(%rdi)\n",        \
      "fstpl (%rsi)", "")                                                                       \
    X(IN_RUN, f_arithp_fstp, M_NONE, IN, "faddp\n", "fstpl (%rsi)", "")                         \
    /* the store of a fused fild m64 / fistp m64 pair: rolled back to the fild */               \
    X(IN_RUN, t_pair_store, M_NONE, IN, "fildll 0x38(%rdi)\n", "fistpll (%rsi)", "")

CASES(DEFINE_CASE)

struct test_case {
    enum group group;
    const char* name;
    enum init init;
    int off;
    void (*fn)(struct io*);
    void (*ref)(struct io*);
    void (*skip)(struct io*);
    const char* fault;
    const char* after;
};

#define TABLE_ENTRY(group, name, init, off, pre, insn, post) \
    {group, #name, init, off, name, name##_ref, name##_skip, name##_fault, name##_after},
static const struct test_case cases[] = {CASES(TABLE_ENTRY)};

/* fnstenv and fnsave images of a small known state, for fldenv and frstor. */
__asm__(".text\n.p2align 4\n_make_images:\n"
        "fninit\n"
        "fldl 0x10(%rsi)\n"
        "fldl 0x18(%rsi)\n"
        "fnstenv (%rdi)\n"
        "fldenv (%rdi)\n"
        "fnsave 0x20(%rdi)\n"
        "ret\n");
extern void make_images(uint8_t* out, struct io* io);
static uint8_t images[160];

enum { REGION = 112, SCRATCH = 256 };
enum mode { MODE_NONE, MODE_RETRY, MODE_SKIP };

/* Two pages: the first stays accessible, the second (page) is the one the
 * cases protect. */
static char* pages;
static char* page;
static char* scratch;
static volatile enum mode mode;
static const char* volatile skip_to;
static volatile int faults;
static struct {
    uint64_t rip;
    uint64_t addr;
    int sig;
    uint16_t fcw, fsw;
    uint8_t ftw;
    uint8_t st[8][10];
} seen;
static int failures;

static void handler(int sig, siginfo_t* info, void* context) {
    ucontext_t* uc = context;
    _STRUCT_X86_FLOAT_STATE64* fs = &uc->uc_mcontext->__fs;
    faults++;
    if (faults == 1) {
        seen.sig = sig;
        seen.rip = uc->uc_mcontext->__ss.__rip;
        seen.addr = (uint64_t)info->si_addr;
        memcpy(&seen.fcw, &fs->__fpu_fcw, 2);
        memcpy(&seen.fsw, &fs->__fpu_fsw, 2);
        seen.ftw = fs->__fpu_ftw;
        for (int i = 0; i < 8; i++)
            memcpy(seen.st[i], (const uint8_t*)&fs->__fpu_stmm0 + 16 * i, 10);
    }
    if (mode == MODE_SKIP && faults == 1)
        uc->uc_mcontext->__ss.__rip = (uint64_t)skip_to;
    else
        mprotect(page, 4096, PROT_READ | PROT_WRITE);
}

static void prepare(struct io* io, const struct test_case* c) {
    memset(io, 0, sizeof(*io));
    io->v[0] = 1.5;
    io->v[1] = -2.25;
    io->v[2] = 3.0;
    io->v[3] = 0.75;
    io->cw = 0x027f;
    io->i32src = 123456;
    io->i64src = 0x123456789aLL;
    io->i16src = -5;
    /* 0.75 in extended precision */
    const uint64_t m = 0xc000000000000000ULL;
    const uint16_t e = 0x3ffe;
    memcpy(io->m80src, &m, 8);
    memcpy(io->m80src + 8, &e, 2);
    io->target = page + c->off;
    io->scratch = scratch;
    memset(scratch, 0xa5, SCRATCH);
    mprotect(page, 4096, PROT_READ | PROT_WRITE);
    memset(pages, 0x5a, 8192);
    char* t = io->target;
    switch (c->init) {
        case M_NONE:
            break;
        case M_F32: {
            const float f = 0.5f;
            memcpy(t, &f, 4);
            break;
        }
        case M_F64: {
            const double d = 0.5;
            memcpy(t, &d, 8);
            break;
        }
        case M_F80: {
            const uint64_t hm = 0x8000000000000000ULL;
            const uint16_t he = 0x3ffe;
            memcpy(t, &hm, 8);
            memcpy(t + 8, &he, 2);
            break;
        }
        case M_I16: {
            const int16_t v = 7;
            memcpy(t, &v, 2);
            break;
        }
        case M_I32: {
            const int32_t v = 7;
            memcpy(t, &v, 4);
            break;
        }
        case M_I64: {
            const int64_t v = 7;
            memcpy(t, &v, 8);
            break;
        }
        case M_BCD:
            memset(t, 0, 10);
            t[0] = 0x42;
            break;
        case M_CW: {
            const uint16_t cw = 0x0a7f;
            memcpy(t, &cw, 2);
            break;
        }
        case M_ENV:
            memcpy(t, images, 28);
            break;
        case M_SAVE:
            memcpy(t, images + 0x20, 108);
            break;
    }
}

struct outcome {
    uint8_t save[108];
    uint8_t region[REGION];
    uint8_t scratch[SCRATCH];
};

static void capture(struct outcome* o, const struct io* io) {
    memcpy(o->save, io->save, 108);
    mprotect(page, 4096, PROT_READ | PROT_WRITE);
    memcpy(o->region, io->target, REGION);
    memcpy(o->scratch, scratch, SCRATCH);
}

static uint16_t u16(const uint8_t* p) {
    uint16_t v;
    memcpy(&v, p, 2);
    return v;
}

static void dump80(const char* what, const uint8_t* p) {
    uint64_t mant;
    memcpy(&mant, p, 8);
    printf("        %s %04x %016llx\n", what, u16(p + 8), (unsigned long long)mant);
}

/* The context at the fault against the reference taken just before the
 * instruction.  Returns the number of mismatches, printed. */
static int check_context(const struct test_case* c, const char* how, const uint8_t* ref,
                         uintptr_t target) {
    int bad = 0;
    const uint16_t fcw = u16(ref + 0), fsw = u16(ref + 4), ftw = u16(ref + 8);
    if (seen.rip != (uint64_t)(uintptr_t)c->fault) {
        printf("FAIL  sync fault %s %s: rip %+lld from the faulting instruction\n", c->name, how,
               (long long)(seen.rip - (uint64_t)(uintptr_t)c->fault));
        bad++;
    }
    if (seen.addr < target || seen.addr >= target + REGION) {
        printf("FAIL  sync fault %s %s: fault address %+lld from the operand\n", c->name, how,
               (long long)(seen.addr - target));
        bad++;
    }
    if (seen.fcw != fcw) {
        printf("FAIL  sync fault %s %s: control word %04x, expected %04x\n", c->name, how,
               seen.fcw, fcw);
        bad++;
    }
    if (seen.fsw != fsw) {
        printf("FAIL  sync fault %s %s: status word %04x, expected %04x\n", c->name, how,
               seen.fsw, fsw);
        bad++;
    }
    uint8_t valid = 0;
    for (int i = 0; i < 8; i++)
        if (((ftw >> (2 * i)) & 3) != 3)
            valid |= (uint8_t)(1u << i);
    if (seen.ftw != valid) {
        printf("FAIL  sync fault %s %s: tag bits %02x, expected %02x\n", c->name, how, seen.ftw,
               valid);
        bad++;
    }
    const int top = (fsw >> 11) & 7;
    for (int k = 0; k < 8; k++) {
        const int phys = (top + k) & 7;
        if (!(valid >> phys & 1))
            continue;
        /* The signal context holds physical slots. */
        if (memcmp(seen.st[phys], ref + 28 + 10 * k, 10) != 0) {
            printf("FAIL  sync fault %s %s: st(%d) differs\n", c->name, how, k);
            dump80("got     ", seen.st[phys]);
            dump80("expected", ref + 28 + 10 * k);
            bad++;
        }
    }
    return bad;
}

static int compare_outcome(const struct test_case* c, const char* how, const struct outcome* got,
                           const struct outcome* want) {
    int bad = 0;
    /* Skip FIP/FCS/FDP/FDS, which name the last instruction, and empty
     * registers, whose contents are left over. */
    int regs_differ = 0;
    const int top = (u16(want->save + 4) >> 11) & 7;
    for (int k = 0; k < 8; k++)
        if (((u16(want->save + 8) >> (2 * ((top + k) & 7))) & 3) != 3 &&
            memcmp(got->save + 28 + 10 * k, want->save + 28 + 10 * k, 10) != 0)
            regs_differ = 1;
    if (memcmp(got->save, want->save, 12) != 0 || regs_differ) {
        printf("FAIL  sync fault %s %s: final x87 state differs (cw %04x/%04x sw %04x/%04x "
               "tw %04x/%04x)\n",
               c->name, how, u16(got->save), u16(want->save), u16(got->save + 4),
               u16(want->save + 4), u16(got->save + 8), u16(want->save + 8));
        bad++;
    }
    /* An fnsave image at the operand holds empty registers too, whose
     * leftover contents the sidecar does not keep across a signal. */
    uint8_t got_region[REGION], want_region[REGION];
    memcpy(got_region, got->region, REGION);
    memcpy(want_region, want->region, REGION);
    if (strstr(c->name, "fnsave") != NULL) {
        const uint16_t tw = u16(want_region + 8);
        const int image_top = (u16(want_region + 4) >> 11) & 7;
        for (int k = 0; k < 8; k++)
            if (((tw >> (2 * ((image_top + k) & 7))) & 3) == 3) {
                memset(got_region + 28 + 10 * k, 0, 10);
                memset(want_region + 28 + 10 * k, 0, 10);
            }
    }
    if (memcmp(got_region, want_region, REGION) != 0) {
        int at = 0;
        while (got_region[at] == want_region[at])
            at++;
        printf("FAIL  sync fault %s %s: memory at the operand differs from byte %d "
               "(%02x, expected %02x)\n",
               c->name, how, at, got_region[at], want_region[at]);
        bad++;
    }
    if (memcmp(got->scratch, want->scratch, SCRATCH) != 0) {
        printf("FAIL  sync fault %s %s: other memory differs\n", c->name, how);
        bad++;
    }
    return bad;
}

static void run_case(const struct test_case* c) {
    static struct io io;
    struct outcome normal, skipped, untouched, got;
    uint8_t ref[108];
    int bad = 0;

    prepare(&io, c);
    mode = MODE_NONE;
    faults = 0;
    c->fn(&io);
    capture(&normal, &io);

    prepare(&io, c);
    c->ref(&io);
    memcpy(ref, io.save, 108);

    prepare(&io, c);
    capture(&untouched, &io);
    c->skip(&io);
    capture(&skipped, &io);
    memcpy(skipped.region, untouched.region, REGION);
    if (faults != 0) {
        printf("FAIL  sync fault %s: %d faults without a protected page\n", c->name, faults);
        bad++;
    }

    static const enum mode modes[] = {MODE_RETRY, MODE_SKIP};
    for (int m = 0; m < 2; m++) {
        const char* how = modes[m] == MODE_RETRY ? "retry" : "skip";
        prepare(&io, c);
        mprotect(page, 4096, PROT_NONE);
        memset(&seen, 0, sizeof(seen));
        faults = 0;
        skip_to = c->after;
        mode = modes[m];
        c->fn(&io);
        mode = MODE_NONE;
        capture(&got, &io);
        if (faults != 1) {
            printf("FAIL  sync fault %s %s: %d faults, expected 1\n", c->name, how, faults);
            bad++;
            if (faults == 0)
                continue;
        }
        bad += check_context(c, how, ref, (uintptr_t)io.target);
        bad += compare_outcome(c, how, &got, modes[m] == MODE_RETRY ? &normal : &skipped);
    }
    if (!bad)
        printf("PASS  sync fault %s (signal %d)\n", c->name, seen.sig);
    failures += bad;
}

int main(void) {
    setvbuf(stdout, NULL, _IOLBF, 0);
    const size_t n = sizeof(cases) / sizeof(cases[0]);
    if (getenv("SYNC_FAULT_LIST")) {
        for (size_t i = 0; i < n; i++)
            printf("%s %s\n", cases[i].name, cases[i].group == PRECISE ? "precise" : "in-run");
        return 0;
    }
    pages = mmap(NULL, 8192, PROT_READ | PROT_WRITE, MAP_ANON | MAP_PRIVATE, -1, 0);
    scratch = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_ANON | MAP_PRIVATE, -1, 0);
    if (pages == MAP_FAILED || scratch == MAP_FAILED) {
        printf("FAIL  sync fault: mmap\n");
        return 1;
    }
    page = pages + 4096;
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = handler;
    sa.sa_flags = SA_SIGINFO;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGSEGV, &sa, NULL) != 0 || sigaction(SIGBUS, &sa, NULL) != 0) {
        printf("FAIL  sync fault: sigaction\n");
        return 1;
    }
    static struct io io;
    prepare(&io, &cases[0]);
    make_images(images, &io);

    const char* only = getenv("SYNC_FAULT_CASE");
    const char* in_run = getenv("SYNC_FAULT_IN_RUN");
    const int all = in_run != NULL && strcmp(in_run, "0") != 0;
    int ran = 0;
    for (size_t i = 0; i < n; i++) {
        if (only ? strcmp(only, cases[i].name) != 0 : !all && cases[i].group != PRECISE)
            continue;
        run_case(&cases[i]);
        ran++;
    }
    if (!ran) {
        printf("FAIL  sync fault: no case named %s\n", only ? only : "");
        return 1;
    }
    printf("%d failure%s\n", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
