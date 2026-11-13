/*
 * x87 code on a page the program writes to must keep giving the results of
 * the code that is there when it runs, as under stock Rosetta.  Rosetta
 * notices writes to pages it has translated code from and translates the
 * code again, so every case below makes it do that once per iteration:
 *
 *   data_write          a byte on the code page, away from the code, is
 *                       written between executions;
 *   self_store_<form>_* the x87 code stores its result into the page that
 *                       holds it, with each store form: as the first x87
 *                       instruction of its run (_first, after an lfence) and
 *                       after the arithmetic in the same run (_arith);
 *   rewrite             the code's fmul is patched to fadd and back between
 *                       executions, and each result must follow the patch;
 *   patch_same/_next    the code patches its own fmul from inside the block
 *                       with an x87 store, in the store's run or in a later
 *                       one, and must execute the patched form;
 *   patch_bridge        the same patch by a 16-bit integer store between two
 *                       x87 instructions, which no bridge takes;
 *   patch_bridged       the same patch by a 32-bit integer store that run
 *                       bridging would join with the x87 instructions around
 *                       it into one run;
 *   patch_len           patch_same's code with the fmul patched to a jump
 *                       over its displacement and back, which moves where
 *                       the block ends on every patch.
 *
 * getpc is not modified code: it checks that fnstenv after an fldz reports
 * the fldz's address as FIP, which packers and shellcode decoders that
 * rewrite themselves use to find their own address.  The other cases that
 * are not modified run code that is never written: file_rx has data_write's
 * code on a private r-x file mapping, the way wine maps a PE section, and
 * file_rx_cross and rwx_cross a loop that crosses a 4 KB boundary, on such a
 * file mapping and on an RWX page.
 *
 * The c_ cases run the same code in 32-bit compatibility mode through an LDT
 * code segment.  Each case copies its code to a fresh mapping below 4 GB
 * (RWX unless noted) and runs it once as a warm-up, in which a patch_ case
 * makes its first real patch, then a fixed number of times with inputs
 * that change every iteration, so a store that did not happen leaves the
 * previous result.  The stored bytes and the status word are compared bit
 * for bit with a reference table taken before the loop from the same code
 * where the linker put it.
 *
 * c_data_write and c_rewrite write the 32-bit code from 64-bit code, which
 * stock Rosetta aborts on with "unsupported privilege level: 0", so they run
 * only when named or with SMC_ALL=1.
 *
 * SMC_CASE=<name> runs one case, SMC_ITERS=<n> sets the iteration count
 * (default 500), SMC_NO_WARMUP=1 leaves out the warm-up (see run_case) and
 * SMC_LIST=1 prints the case names.
 */
#include <architecture/i386/desc.h>
#include <architecture/i386/table.h>
#include <errno.h>
#include <i386/user_ldt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

/* Layout shared with the assembly.  The code reaches the inputs and the
 * status word through the pointer at 0x30 of its output area, so a case
 * whose output area is on the code page writes nothing else there. */
struct area {
    double a;        /* 0x00 */
    double b;        /* 0x08 */
    uint8_t out[16]; /* 0x10 */
    uint16_t sw;     /* 0x20 */
    uint8_t pad[6];  /* 0x22 */
    int32_t patch;   /* 0x28, the value a patch_ case stores */
    uint8_t pad2[4]; /* 0x2c */
    struct area* in; /* 0x30 */
    uint8_t env[28]; /* 0x38, fnstenv image */
};
_Static_assert(__builtin_offsetof(struct area, out) == 0x10, "area layout");
_Static_assert(__builtin_offsetof(struct area, sw) == 0x20, "area layout");
_Static_assert(__builtin_offsetof(struct area, patch) == 0x28, "area layout");
_Static_assert(__builtin_offsetof(struct area, in) == 0x30, "area layout");
_Static_assert(__builtin_offsetof(struct area, env) == 0x38, "area layout");

#define BODY64(store)        \
    "mov 0x30(%rdi), %rsi\n" \
    "fninit\n"               \
    "fldl 0x00(%rsi)\n"      \
    "fmull 0x08(%rsi)\n" store "\nfnstsw 0x20(%rsi)\n"

#define BODY32(store)        \
    "mov 0x30(%eax), %edx\n" \
    "fninit\n"               \
    "fldl 0x00(%edx)\n"      \
    "fmull 0x08(%edx)\n" store "\nfnstsw 0x20(%edx)\n"

/* 64-bit code is called with the output area in RDI.  32-bit code is
 * entered by a far jump with it in EAX and left by a far return to CS:EIP
 * held in EDI:ESI. */
#define DEFINE64(name, body)                                                                 \
    __asm__(".text\n.p2align 4\n.globl _" #name "\n_" #name ":\n" body "ret\n.globl _" #name \
            "_end\n_" #name "_end:\n");                                                      \
    extern char name[], name##_end[];
#define DEFINE32(name, body)                                                                  \
    __asm__(".text\n.p2align 4\n.code32\n.globl _" #name "\n_" #name ":\n" body               \
            "push %edi\npush %esi\nlret\n.globl _" #name "_end\n_" #name "_end:\n.code64\n"); \
    extern char name[], name##_end[];

/* kind: what the loop does between executions. */
enum kind { K_DATA, K_SELF, K_REWRITE, K_PATCH, K_GETPC, K_STILL };

/* form, store width, store instruction (64-bit and 32-bit operand). */
#define STORES(X)                                             \
    X(fstp32, 4, "fstps 0x10(%rdi)", "fstps 0x10(%eax)")      \
    X(fstp64, 8, "fstpl 0x10(%rdi)", "fstpl 0x10(%eax)")      \
    X(fistp32, 4, "fistpl 0x10(%rdi)", "fistpl 0x10(%eax)")   \
    X(fistp64, 8, "fistpll 0x10(%rdi)", "fistpll 0x10(%eax)") \
    X(fstp80, 10, "fstpt 0x10(%rdi)", "fstpt 0x10(%eax)")

DEFINE64(data_write, BODY64("fstpl 0x10(%rdi)"))
DEFINE32(c_data_write, BODY32("fstpl 0x10(%eax)"))

#define DEFINE_SELF(form, width, s64, s32)                        \
    DEFINE64(self_store_##form##_first, BODY64("lfence\n" s64))   \
    DEFINE64(self_store_##form##_arith, BODY64(s64))              \
    DEFINE32(c_self_store_##form##_first, BODY32("lfence\n" s32)) \
    DEFINE32(c_self_store_##form##_arith, BODY32(s32))
STORES(DEFINE_SELF)

/* The rewrite cases patch the ModRM byte of the instruction at the
 * _patch label: fmull 8(%rsi) is DC 4E 08, faddl 8(%rsi) DC 46 08, and the
 * same holds for %edx with 4A and 42.  The _add twins give the reference
 * for the patched form. */
__asm__(
    ".text\n.p2align 4\n.globl _rewrite\n_rewrite:\n"
    "mov 0x30(%rdi), %rsi\nfninit\nfldl 0x00(%rsi)\n"
    ".globl _rewrite_patch\n_rewrite_patch:\nfmull 0x08(%rsi)\n"
    "fstpl 0x10(%rdi)\nfnstsw 0x20(%rsi)\nret\n.globl _rewrite_end\n_rewrite_end:\n"
    ".p2align 4\n.globl _rewrite_add\n_rewrite_add:\n"
    "mov 0x30(%rdi), %rsi\nfninit\nfldl 0x00(%rsi)\nfaddl 0x08(%rsi)\n"
    "fstpl 0x10(%rdi)\nfnstsw 0x20(%rsi)\nret\n_rewrite_add_end:\n"
    ".p2align 4\n.code32\n.globl _c_rewrite\n_c_rewrite:\n"
    "mov 0x30(%eax), %edx\nfninit\nfldl 0x00(%edx)\n"
    ".globl _c_rewrite_patch\n_c_rewrite_patch:\nfmull 0x08(%edx)\n"
    "fstpl 0x10(%eax)\nfnstsw 0x20(%edx)\npush %edi\npush %esi\nlret\n"
    ".globl _c_rewrite_end\n_c_rewrite_end:\n"
    ".p2align 4\n.globl _c_rewrite_add\n_c_rewrite_add:\n"
    "mov 0x30(%eax), %edx\nfninit\nfldl 0x00(%edx)\nfaddl 0x08(%edx)\n"
    "fstpl 0x10(%eax)\nfnstsw 0x20(%edx)\npush %edi\npush %esi\nlret\n_c_rewrite_add_end:\n"
    ".code64\n");
extern char rewrite[], rewrite_end[], rewrite_patch[], rewrite_add[];
extern char c_rewrite[], c_rewrite_end[], c_rewrite_patch[], c_rewrite_add[];

/* The reference for patch_len's jump form: the patched fmull is skipped. */
__asm__(
    ".text\n.p2align 4\n.globl _rewrite_skip\n_rewrite_skip:\n"
    "mov 0x30(%rdi), %rsi\nfninit\nfldl 0x00(%rsi)\n"
    "fstpl 0x10(%rdi)\nfnstsw 0x20(%rsi)\nret\n"
    ".p2align 4\n.code32\n.globl _c_rewrite_skip\n_c_rewrite_skip:\n"
    "mov 0x30(%eax), %edx\nfninit\nfldl 0x00(%edx)\n"
    "fstpl 0x10(%eax)\nfnstsw 0x20(%edx)\npush %edi\npush %esi\nlret\n"
    ".code64\n");
extern char rewrite_skip[], c_rewrite_skip[];

/* The patch_ cases rewrite their own fmull into faddl from inside the run:
 * an fistps of 0x4edc or 0x46dc writes DC 4E (fmul) or DC 46 (fadd) over
 * the opcode and ModRM of an instruction further down the same block,
 * which x86 then executes in its new form.  _same keeps the store and the
 * patched instruction in one x87 run, _next ends the run in between.  The
 * 32-bit forms find their own address with call/pop and patch DC 4A or
 * DC 42.  Their results are those of rewrite and rewrite_add. */
#define PATCH64(gap)                                   \
    "mov 0x30(%rdi), %rsi\nfninit\nfildl 0x28(%rsi)\n" \
    "fistps 1f(%rip)\n" gap                            \
    "fldl 0x00(%rsi)\n"                                \
    "1:\nfmull 0x08(%rsi)\nfstpl 0x10(%rdi)\nfnstsw 0x20(%rsi)\n"
#define PATCH32(gap)                                        \
    "mov 0x30(%eax), %edx\ncall 2f\n2:\npop %ecx\nfninit\n" \
    "fildl 0x28(%edx)\nfistps (1f-2b)(%ecx)\n" gap          \
    "fldl 0x00(%edx)\n"                                     \
    "1:\nfmull 0x08(%edx)\nfstpl 0x10(%eax)\nfnstsw 0x20(%edx)\n"
DEFINE64(patch_same, PATCH64(""))
/* The same patch by a 16-bit integer store between two x87 instructions.
 * Run bridging takes only 32- and 64-bit moves, so the store stays stock's
 * and ends the run. */
DEFINE64(patch_bridge,
         "mov 0x30(%rdi), %rsi\nmovl 0x28(%rsi), %ecx\nfninit\nfldl 0x00(%rsi)\n"
         "movw %cx, 1f(%rip)\n1:\nfmull 0x08(%rsi)\nfstpl 0x10(%rdi)\nfnstsw 0x20(%rsi)\n")
DEFINE64(patch_next, PATCH64("lfence\n"))
DEFINE32(c_patch_same, PATCH32(""))
DEFINE32(c_patch_next, PATCH32("lfence\n"))
/* The patch by a 32-bit store through a register, a mov run bridging takes:
 * it writes DC 4E 08 DD (fmul) or DC 46 08 DD (fadd) over the fmull and the
 * first byte of the fstpl after it, which is DD in both modes.  The 32-bit
 * form patches DC 4A 08 DD or DC 42 08 DD.  The lfence keeps the fnstsw,
 * which the bridged lowering does not take, out of the bridged region. */
DEFINE64(patch_bridged,
         "mov 0x30(%rdi), %rsi\nlea 1f(%rip), %rdx\nmovl 0x28(%rsi), %ecx\nfninit\n"
         "fldl 0x00(%rsi)\nmovl %ecx, (%rdx)\n1:\nfmull 0x08(%rsi)\nfstpl 0x10(%rdi)\n"
         "lfence\nfnstsw 0x20(%rsi)\n")
DEFINE32(c_patch_bridged,
         "mov 0x30(%eax), %edx\ncall 2f\n2:\npop %ecx\nlea (1f-2b)(%ecx), %ecx\n"
         "movl 0x28(%edx), %ebx\nfninit\nfldl 0x00(%edx)\nmovl %ebx, (%ecx)\n"
         "1:\nfmull 0x08(%edx)\nfstpl 0x10(%eax)\nlfence\nfnstsw 0x20(%edx)\n")

/* fldz followed by fnstenv: the environment's FIP names the fldz, which
 * position-independent x86 code (packers, shellcode decoders) uses to find
 * its own address. */
__asm__(
    ".text\n.p2align 4\n.globl _getpc\n_getpc:\n"
    "mov 0x30(%rdi), %rsi\nfninit\n.globl _getpc_insn\n_getpc_insn:\nfldz\n"
    "fnstenv 0x38(%rsi)\nfstp %st(0)\nret\n.globl _getpc_end\n_getpc_end:\n"
    ".p2align 4\n.code32\n.globl _c_getpc\n_c_getpc:\n"
    "mov 0x30(%eax), %edx\nfninit\n.globl _c_getpc_insn\n_c_getpc_insn:\nfldz\n"
    "fnstenv 0x38(%edx)\nfstp %st(0)\npush %edi\npush %esi\nlret\n"
    ".globl _c_getpc_end\n_c_getpc_end:\n.code64\n");
extern char getpc[], getpc_end[], getpc_insn[], c_getpc[], c_getpc_end[], c_getpc_insn[];

/* A loop of 15 x87 instructions with stores inside its run, placed so that
 * it crosses a 4 KB boundary.  Rosetta ends a block at the boundary, so the
 * loop head starts a block of its own in a second translation; the code is
 * never written, and must keep its runs.  Its inputs are in the output area
 * itself (the case's area is its own input), and out gets (a*b + a)*b + a
 * and that times b. */
DEFINE64(cross_loop,
         "push $4\npop %rcx\n"
         "2:\n"
         "fldl 0x00(%rdi)\nfmull 0x08(%rdi)\nfstpl 0x18(%rdi)\n"
         "fldl 0x18(%rdi)\nfaddl 0x00(%rdi)\nfstpl 0x10(%rdi)\n"
         "fldl 0x10(%rdi)\nfmull 0x08(%rdi)\nfstpl 0x18(%rdi)\n"
         "fldl 0x18(%rdi)\nfaddl 0x00(%rdi)\nfstpl 0x10(%rdi)\n"
         "fldl 0x10(%rdi)\nfmull 0x08(%rdi)\nfstpl 0x18(%rdi)\n"
         "dec %ecx\njnz 2b\nfnstsw 0x20(%rdi)\n")
/* The loop head lands 3 bytes in, at 0xfe2, so its run crosses 0x1000. */
#define CROSS_OFF 0xfdf

struct test_case {
    const char* name;
    enum kind kind;
    int width;
    int compat;
    const char* code;
    const char* code_end;
    const char* patch; /* rewrite: the patched instruction; getpc: the fldz */
    const char* ref;   /* reference for the code as built, if not code */
    const char* alt;   /* rewrite, patch_: the reference for the patched form */
    uint32_t word_hi;  /* patch_: bytes the store writes past the opcode and ModRM */
    uint32_t alt_word; /* patch_: the patched form's bytes, if not the fadd */
    int stock_aborts;  /* runs only when named or with SMC_ALL=1 */
    int offset;        /* where the code starts in its two-page mapping */
    int file;          /* map a temporary file r-x instead of anonymous RWX */
};

#define CODE(sym) .code = sym, .code_end = sym##_end
#define SELF1(prefix, form, when, w, mode)          \
    {.name = #prefix "self_store_" #form "_" #when, \
     .kind = K_SELF,                                \
     .width = w,                                    \
     .compat = mode,                                \
     CODE(prefix##self_store_##form##_##when)}
#define SELF(prefix, form, w, mode) \
    SELF1(prefix, form, first, w, mode), SELF1(prefix, form, arith, w, mode)
#define PATCH(sym, mode, refs) \
    .name = #sym, .kind = K_PATCH, .width = 8, .compat = mode, CODE(sym), refs
#define REFS64 .ref = rewrite, .alt = rewrite_add
#define REFS32 .ref = c_rewrite, .alt = c_rewrite_add

/* patch_bridged's store also writes the fmull's disp8 and the fstpl's opcode. */
#define DISP8_FSTP 0xdd080000u
/* patch_len's patched form: EB 01, a jump over the fmull's disp8, which
 * ends the block at the jump and so changes where it ends. */
#define JMP_OVER_DISP8 0x01ebu

static const struct test_case cases[] = {
    {.name = "data_write", .kind = K_DATA, .width = 8, CODE(data_write)},
    SELF(, fstp32, 4, 0),
    SELF(, fstp64, 8, 0),
    SELF(, fistp32, 4, 0),
    SELF(, fistp64, 8, 0),
    SELF(, fstp80, 10, 0),
    {.name = "rewrite",
     .kind = K_REWRITE,
     .width = 8,
     CODE(rewrite),
     .patch = rewrite_patch,
     .alt = rewrite_add},
    {PATCH(patch_same, 0, REFS64)},
    {PATCH(patch_next, 0, REFS64)},
    {PATCH(patch_bridge, 0, REFS64)},
    {PATCH(patch_bridged, 0, REFS64), .word_hi = DISP8_FSTP},
    {.name = "patch_len",
     .kind = K_PATCH,
     .width = 8,
     CODE(patch_same),
     .ref = rewrite,
     .alt = rewrite_skip,
     .alt_word = JMP_OVER_DISP8},
    {.name = "getpc", .kind = K_GETPC, .width = 4, CODE(getpc), .patch = getpc_insn},
    {.name = "file_rx", .kind = K_STILL, .width = 8, CODE(data_write), .file = 1},
    {.name = "file_rx_cross",
     .kind = K_STILL,
     .width = 16,
     CODE(cross_loop),
     .offset = CROSS_OFF,
     .file = 1},
    {.name = "rwx_cross", .kind = K_STILL, .width = 16, CODE(cross_loop), .offset = CROSS_OFF},
    {.name = "c_data_write",
     .kind = K_DATA,
     .width = 8,
     .compat = 1,
     CODE(c_data_write),
     .stock_aborts = 1},
    SELF(c_, fstp32, 4, 1),
    SELF(c_, fstp64, 8, 1),
    SELF(c_, fistp32, 4, 1),
    SELF(c_, fistp64, 8, 1),
    SELF(c_, fstp80, 10, 1),
    {.name = "c_rewrite",
     .kind = K_REWRITE,
     .width = 8,
     .compat = 1,
     CODE(c_rewrite),
     .patch = c_rewrite_patch,
     .alt = c_rewrite_add,
     .stock_aborts = 1},
    {PATCH(c_patch_same, 1, REFS32)},
    {PATCH(c_patch_next, 1, REFS32)},
    {PATCH(c_patch_bridged, 1, REFS32), .word_hi = DISP8_FSTP},
    {.name = "c_patch_len",
     .kind = K_PATCH,
     .width = 8,
     .compat = 1,
     CODE(c_patch_same),
     .ref = c_rewrite,
     .alt = c_rewrite_skip,
     .alt_word = JMP_OVER_DISP8},
    {.name = "c_getpc",
     .kind = K_GETPC,
     .width = 4,
     .compat = 1,
     CODE(c_getpc),
     .patch = c_getpc_insn},
};

static int failures;

/* ── 32-bit compatibility mode, as in test_x87_sync_fault ──────────────── */

static uint16_t sel_cs32, sel_ds32, sel_cs64;

static uint16_t alloc_ldt_seg(int idx, int is_code) {
    ldt_entry_t e;
    memset(&e, 0, sizeof e);
    if (is_code) {
        e.code.limit00 = 0xFFFF;
        e.code.type = DESC_CODE_READ;
        e.code.dpl = 3;
        e.code.present = 1;
        e.code.limit16 = 0xF;
        e.code.opsz = DESC_CODE_32B;
        e.code.granular = DESC_GRAN_PAGE;
    } else {
        e.data.limit00 = 0xFFFF;
        e.data.type = DESC_DATA_WRITE;
        e.data.dpl = 3;
        e.data.present = 1;
        e.data.limit16 = 0xF;
        e.data.stksz = DESC_DATA_32B;
        e.data.granular = DESC_GRAN_PAGE;
    }
    if (i386_set_ldt(idx, &e, 1) < 0) {
        printf("FAIL  smc: i386_set_ldt(%d) failed (errno=%d %s)\n", idx, errno, strerror(errno));
        failures++;
        return 0;
    }
    return (uint16_t)((idx << 3) | (1 << 2) | 3);
}

struct __attribute__((packed)) farptr32 {
    uint32_t off;
    uint16_t sel;
};

static void call32(const char* entry, struct area* area) {
    struct farptr32 to32 = {(uint32_t)(uintptr_t)entry, sel_cs32};
    uint64_t in_eax = (uintptr_t)area;
    uint32_t in_ds32 = sel_ds32, in_cs64 = sel_cs64;
    __asm__ volatile(
        "leaq 1f(%%rip), %%rsi   \n"
        "movl %k[incs], %%edi    \n"
        "mov  %%ds, %%r10w       \n"
        "mov  %%es, %%r8w        \n"
        "mov  %%ss, %%r9w        \n"
        "movl %k[inds], %%eax    \n"
        "mov  %%ax, %%ds         \n"
        "mov  %%ax, %%es         \n"
        "mov  %%ax, %%ss         \n"
        "movl %k[ineax], %%eax   \n"
        "ljmp *%[fp]             \n"
        "1:                      \n"
        "mov  %%r10w, %%ds       \n"
        "mov  %%r8w, %%es        \n"
        "mov  %%r9w, %%ss        \n"
        :
        : [ineax] "r"(in_eax), [inds] "r"(in_ds32), [incs] "r"(in_cs64), [fp] "m"(to32)
        : "rax", "rbx", "rcx", "rdx", "rsi", "rdi", "r8", "r9", "r10", "cc", "memory", "st",
          "st(1)", "st(2)", "st(3)", "st(4)", "st(5)", "st(6)", "st(7)");
}

static void invoke(const struct test_case* c, const char* entry, struct area* area) {
    if (c->compat)
        call32(entry, area);
    else
        ((void (*)(struct area*))(uintptr_t)entry)(area);
}

/* ── the loop ──────────────────────────────────────────────────────────── */

enum {
    PERIOD = 16,
    CODE_PAGE = 4096,
    MAP_LEN = 2 * CODE_PAGE,
    AREA_OFF = 0x800,
    DATA_BYTE_OFF = 0xc00
};

struct result {
    uint8_t out[16];
    uint16_t sw;
};

/* Inputs for iteration i: a product of 3 + 2k and a sum of 3.5 + k, both
 * exact in every store form. */
static void set_inputs(struct area* in, unsigned i) {
    in->a = 1.5 + (double)(i % PERIOD);
    in->b = 2.0;
}

static double now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e9 + (double)ts.tv_nsec;
}

static void reference(const struct test_case* c, const char* entry, struct result* ref) {
    static struct area ref_area;
    for (unsigned k = 0; k < PERIOD; k++) {
        memset(&ref_area, 0xa5, sizeof(ref_area));
        ref_area.in = &ref_area;
        set_inputs(&ref_area, k);
        invoke(c, entry, &ref_area);
        memcpy(ref[k].out, ref_area.out, 16);
        ref[k].sw = ref_area.sw;
    }
}

static void dump(const char* what, const uint8_t* p, int width, uint16_t sw) {
    printf("        %s", what);
    for (int b = width - 1; b >= 0; b--)
        printf("%02x", p[b]);
    printf(" sw %04x\n", sw);
}

static int next_hint;
static int warmup = 1;

/* A fresh two-page mapping per case with the code at c->offset, below 4 GB
 * so 32-bit code can run there: RWX and anonymous, or with c->file a private
 * r-x mapping of a temporary file, the way wine maps a PE section. */
static char* map_code(const struct test_case* c) {
    static char buf[MAP_LEN];
    memset(buf, 0xcc, sizeof(buf));
    memcpy(buf + c->offset, c->code, (size_t)(c->code_end - c->code));
    void* hint = (void*)(uintptr_t)(0x20000000u + 0x100000u * (unsigned)next_hint++);
    char* page = MAP_FAILED;
    if (c->file) {
        char path[1024];
        const char* dir = getenv("TMPDIR");
        snprintf(path, sizeof(path), "%s/test_x87_smc.XXXXXX", dir ? dir : "/tmp");
        const int fd = mkstemp(path);
        if (fd < 0 || write(fd, buf, sizeof(buf)) != (ssize_t)sizeof(buf)) {
            printf("FAIL  smc %s: cannot write %s (errno=%d)\n", c->name, path, errno);
            if (fd >= 0)
                close(fd);
            return NULL;
        }
        page = mmap(hint, MAP_LEN, PROT_READ | PROT_EXEC, MAP_PRIVATE, fd, 0);
        unlink(path);
        close(fd);
    } else {
        page =
            mmap(hint, MAP_LEN, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANON, -1, 0);
        if (page != MAP_FAILED)
            memcpy(page, buf, sizeof(buf));
    }
    if (page == MAP_FAILED || (uintptr_t)page + MAP_LEN > 0x100000000ull) {
        printf("FAIL  smc %s: no code page below 4 GB (%p)\n", c->name, (void*)page);
        return NULL;
    }
    return page;
}

static void run_case(const struct test_case* c, unsigned iters) {
    struct result ref[2][PERIOD];
    reference(c, c->ref ? c->ref : c->code, ref[0]);
    if (c->alt)
        reference(c, c->alt, ref[1]);
    /* Rosetta may not keep FIP at all; the copy is then held to the same. */
    int fip_kept = 1;
    if (c->kind == K_GETPC) {
        uint32_t fip;
        static struct area ref_area;
        memset(&ref_area, 0, sizeof(ref_area));
        ref_area.in = &ref_area;
        invoke(c, c->code, &ref_area);
        memcpy(&fip, ref_area.env + 12, 4);
        if (fip != (uint32_t)(uintptr_t)c->patch) {
            printf("NOTE  smc %s: FIP %#x where the binary has the fldz at %#x\n", c->name, fip,
                   (uint32_t)(uintptr_t)c->patch);
            fip_kept = fip != 0;
        }
    }

    char* page = map_code(c);
    if (page == NULL) {
        failures++;
        return;
    }
    char* const entry = page + c->offset;

    static struct area side;
    memset(&side, 0, sizeof(side));
    side.in = &side;
    struct area* out = &side;
    if (c->kind == K_SELF) {
        out = (struct area*)(page + AREA_OFF);
        memset(out, 0xa5, sizeof(*out));
        out->in = &side;
    }
    size_t patch_off = 0;
    if (c->kind == K_REWRITE) {
        patch_off = (size_t)(c->patch - c->code) + 1;
        if ((uint8_t)entry[patch_off - 1] != 0xdc || ((uint8_t)entry[patch_off] & 0x38) != 0x08) {
            printf("FAIL  smc %s: unexpected encoding %02x %02x at the patch\n", c->name,
                   (uint8_t)entry[patch_off - 1], (uint8_t)entry[patch_off]);
            failures++;
            munmap(page, MAP_LEN);
            return;
        }
    }
    const uint8_t modrm = (uint8_t)entry[patch_off];
    if (c->kind == K_GETPC) {
        const uint32_t fip = fip_kept ? (uint32_t)(uintptr_t)(entry + (c->patch - c->code)) : 0;
        for (unsigned k = 0; k < PERIOD; k++) {
            memset(&ref[0][k], 0, sizeof(ref[0][k]));
            memcpy(ref[0][k].out, &fip, 4);
        }
    }
    /* The stored value whose low bytes are DC and the fmul or fadd ModRM. */
    const uint32_t patched = c->alt_word ? c->alt_word : (c->compat ? 0x42dcu : 0x46dcu);
    const int32_t patch_word[2] = {(int32_t)((c->compat ? 0x4adcu : 0x4edcu) | c->word_hi),
                                   (int32_t)(patched | c->word_hi)};

    /* The warm-up runs the code once before the measured loop, a patch_ case
     * writing its patched form over the fmul (fadd, or for patch_len the
     * jump).  That execution still runs the fmul, and its
     * result is not checked: the sidecar learns that the page is rewritten
     * only from the next translation, which finds the new bytes (see
     * docs/internals.md, "Code on writable pages").  SMC_NO_WARMUP=1 leaves
     * it out, so the loop's first iteration is that execution. */
    if (warmup) {
        set_inputs(&side, 0);
        side.patch = patch_word[1];
        invoke(c, entry, out);
    }
    const unsigned op_base = warmup ? 0 : 1;

    unsigned bad = 0;
    long first_bad = -1;
    struct result got_first = {{0}, 0};
    int first_op = 0;
    const double t0 = now_ns();
    for (unsigned i = 0; i < iters; i++) {
        const unsigned k = i % PERIOD;
        int op = 0;
        set_inputs(&side, i);
        side.sw = 0xdead;
        if (c->kind != K_SELF)
            memset(side.out, 0xa5, sizeof(side.out));
        if (c->kind == K_DATA)
            page[DATA_BYTE_OFF] = (char)i;
        if (c->kind == K_REWRITE) {
            op = (int)(i & 1);
            entry[patch_off] = (char)(op ? modrm & ~0x08 : modrm);
        }
        if (c->kind == K_PATCH) {
            op = (int)((i + op_base) & 1);
            side.patch = patch_word[op];
        }
        invoke(c, entry, out);
        struct result got;
        if (c->kind == K_GETPC) {
            memset(&got, 0, sizeof(got));
            memcpy(got.out, side.env + 12, 4);
        } else {
            memcpy(got.out, out->out, 16);
            got.sw = side.sw;
        }
        const struct result* want = &ref[op][k];
        if (memcmp(got.out, want->out, (size_t)c->width) != 0 || got.sw != want->sw) {
            if (bad++ == 0) {
                first_bad = (long)i;
                got_first = got;
                first_op = op;
            }
        }
    }
    const double per = (now_ns() - t0) / (double)iters;
    if (bad) {
        printf("FAIL  smc %s: %u of %u iterations differ (%.0f ns/iter)\n", c->name, bad, iters,
               per);
        printf("      first at iteration %ld%s\n", first_bad,
               c->kind == K_REWRITE || c->kind == K_PATCH ? (first_op ? " (patched)" : " (fmul)")
                                                          : "");
        dump("got      ", got_first.out, c->width, got_first.sw);
        const struct result* want = &ref[first_op][first_bad % PERIOD];
        dump("expected ", want->out, c->width, want->sw);
        failures++;
    } else {
        printf("PASS  smc %s (%u iterations, %.0f ns/iter)\n", c->name, iters, per);
    }
    munmap(page, MAP_LEN);
}

int main(void) {
    setvbuf(stdout, NULL, _IOLBF, 0);
    const size_t n = sizeof(cases) / sizeof(cases[0]);
    if (getenv("SMC_LIST")) {
        for (size_t i = 0; i < n; i++)
            printf("%s\n", cases[i].name);
        return 0;
    }
    unsigned iters = 500;
    const char* it = getenv("SMC_ITERS");
    if (it != NULL && atoi(it) > 0)
        iters = (unsigned)atoi(it);
    const int all = getenv("SMC_ALL") != NULL;
    warmup = getenv("SMC_NO_WARMUP") == NULL;
    sel_cs32 = alloc_ldt_seg(3, 1);
    sel_ds32 = alloc_ldt_seg(4, 0);
    __asm__ volatile("mov %%cs, %0" : "=r"(sel_cs64));
    if (failures)
        return 1;

    const char* only = getenv("SMC_CASE");
    int ran = 0;
    for (size_t i = 0; i < n; i++) {
        if (only && strcmp(only, cases[i].name) != 0)
            continue;
        if (!only && !all && cases[i].stock_aborts)
            continue;
        run_case(&cases[i], iters);
        ran++;
    }
    if (!ran) {
        printf("FAIL  smc: no case named %s\n", only ? only : "");
        return 1;
    }
    printf("%d failure%s\n", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
