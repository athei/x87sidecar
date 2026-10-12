/*
 * An x87 memory access that faults must reach the program's SIGSEGV/SIGBUS
 * handler with a precise context, as under stock Rosetta: rip at the faulting
 * instruction, the fault address and kind, and the x87 stack, TOP, status
 * word and control word as they were before it.  Windows programs under
 * wine rely on this through structured exception handling (guard pages,
 * write watches, IsBadReadPtr, deliberate access violations).
 *
 * Every case runs one x87 instruction against a protected page: alone in
 * its translation reply, at the start of a longer x87 run (also one joined
 * across a mov by bridging), as the first half of a two-instruction run that
 * a fusion takes, in a run of integer loads and stores only, and straddling
 * the page boundary.  The page is PROT_NONE, or read-only for stores (the
 * write-watch path).  Operands are a bare register, a displacement, an
 * index, RIP-relative and %gs-relative, and a set of cases runs in 32-bit
 * compatibility mode through an LDT code segment, as wine's 32-bit programs
 * do.  The handler checks the context against a reference taken just before
 * the instruction, then either makes the page accessible and returns, so
 * the instruction runs again, or moves rip past the instruction.  The final
 * x87 state and memory are compared with an unfaulted run and with a run
 * that leaves the instruction out.
 *
 * Fault kind: stock reports a load as a read fault (__err 0x4) and a store
 * as a write fault (0x6).  The sidecar probes a store at the start of a
 * reply with a load and a write-back, so a store into a PROT_NONE page that
 * it probes is reported as a read fault.  Those cases (E_WP) accept either;
 * every other case must match stock.
 *
 * A last group stores through x87 instructions into write-combined and
 * uncached IOSurface memory, where LSE atomics take a bus error and plain
 * loads and stores work.
 *
 * The sidecar answers a run of floating-point x87 instructions with one
 * reply, and the runtime can only roll a fault back to the start of a
 * reply, so a fault in an instruction that is not the first of its reply
 * still aborts the process (docs/internals.md, "Memory faults inside
 * emitted code"), and the store of a fused fild m64 / fistp m64 pair is
 * rolled back to the fild instead.  Those cases are in a second group that
 * runs only with SYNC_FAULT_IN_RUN=1.  They pass under stock Rosetta, and
 * under x87sidecar with X87_NATIVE_SLOTS=1 as well, which ends a reply
 * before each later memory access when X87_PRECISE_FAULTS=1.
 *
 * SYNC_FAULT_CASE=<name> runs one case of either group (wc for the
 * IOSurface group); SYNC_FAULT_LIST=1 prints every name with its group.
 */
#include <IOSurface/IOSurface.h>
#include <architecture/i386/desc.h>
#include <architecture/i386/table.h>
#include <errno.h>
#include <i386/user_ldt.h>
#include <pthread.h>
#include <setjmp.h>
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
    int16_t pad1[3];      /* 0xc2 */
    int64_t gsoff;        /* 0xc8, the target relative to the %gs base */
};
_Static_assert(__builtin_offsetof(struct io, save) == 0x50, "io layout");
_Static_assert(__builtin_offsetof(struct io, i16src) == 0xc0, "io layout");
_Static_assert(__builtin_offsetof(struct io, gsoff) == 0xc8, "io layout");

/* Three pages: the middle one is the one the cases protect.  The binary is
 * linked below 4 GB, so 32-bit code reaches all of it. */
char g_pages[3 * 4096] __attribute__((aligned(4096)));
static char scratch_area[4096] __attribute__((aligned(4096)));

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

#define DEFINE_CASE(group, name, init, off, prot, err, pre, insn, post)                       \
    __asm__(".text\n.p2align 4\n.globl _" #name "\n_" #name ":\n" SETUP pre                   \
            "\n.globl _" #name "_fault\n_" #name "_fault:\n" insn "\n.globl _" #name            \
            "_after\n_" #name "_after:\n" post FINISH ".p2align 4\n.globl _" #name              \
            "_ref\n_" #name "_ref:\n" SETUP pre FINISH ".p2align 4\n.globl _" #name "_skip\n_" \
            #name "_skip:\n" SETUP pre "\n" post FINISH);                                      \
    extern char name[], name##_ref[], name##_skip[], name##_fault[], name##_after[];

/* The same in 32-bit code, entered by a far jump with the io block in EAX
 * and left by a far return to CS:EIP held in EDI:ESI. */
#define SETUP32                       \
    "fninit\n"                        \
    "fldcw 0x20(%eax)\n"              \
    "mov 0x28(%eax), %ebx\n"          \
    "mov 0x30(%eax), %edx\n"          \
    "fldl 0x00(%eax)\n"               \
    "fldl 0x08(%eax)\n"               \
    "lfence\n"
#define FINISH32          \
    "\nlfence\n"          \
    "fnsave 0x50(%eax)\n" \
    "push %edi\n"         \
    "push %esi\n"         \
    "lret\n"

#define DEFINE_CASE32(group, name, init, off, prot, err, pre, insn, post)                     \
    __asm__(".text\n.p2align 4\n.code32\n.globl _" #name "\n_" #name ":\n" SETUP32 pre        \
            "\n.globl _" #name "_fault\n_" #name "_fault:\n" insn "\n.globl _" #name            \
            "_after\n_" #name "_after:\n" post FINISH32 ".p2align 4\n.globl _" #name            \
            "_ref\n_" #name "_ref:\n" SETUP32 pre FINISH32 ".p2align 4\n.globl _" #name         \
            "_skip\n_" #name "_skip:\n" SETUP32 pre "\n" post FINISH32 ".code64\n");          \
    extern char name[], name##_ref[], name##_skip[], name##_fault[], name##_after[];

enum init { M_NONE, M_F32, M_F64, M_F80, M_I16, M_I32, M_I64, M_BCD, M_CW, M_ENV, M_SAVE };

/* Groups: PRECISE must pass everywhere; IN_RUN is precise under the sidecar
 * only with X87_NATIVE_SLOTS=1 and runs only on request. */
enum group { PRECISE, IN_RUN };

/* Protection of the target page while the case runs. */
enum prot { P_NONE, P_RO };

/* Fault kind expected: a read, a write, or a write that the sidecar's
 * store probe reports as a read when the page is PROT_NONE. */
enum err { E_RD, E_WR, E_WP };

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
/* The same length for a stack one shorter, after a popping store. */
#define RUN_POST_POP "fldl 0x18(%rdi)\nfmul %st(1), %st\nfadd %st(1), %st\n"
#define BRIDGE_POST "fmull 0x10(%rdi)\nmov 0x38(%rdi), %rcx\nfaddl 0x18(%rdi)\nfstpl (%rdx)\n"
/* Integer loads and stores only: a native transport run.  TR_PRE ends in
 * m16 loads so that an fistpll after it is not a fused copy pair, and so
 * that TR_POST's 32-bit store finds a small value whether or not the
 * instruction under test ran. */
#define TR_PRE "fildll 0x38(%rdi)\nfilds 0xc0(%rdi)\nfilds 0xc0(%rdi)\n"
#define TR_POST "fistpl 8(%rdx)\nfildl 0x24(%rdi)\nfistpll 16(%rdx)\n"
/* Other operand forms, all reaching the target: r8 + 0x40,
 * r8 + 4 * rcx + 8, the protected page through RIP, and %gs:rcx. */
#define DISP_PRE "lea -0x40(%rsi), %r8\n"
#define INDEX_PRE "mov $0x10, %ecx\nlea -0x48(%rsi), %r8\n"
#define GS_PRE "mov 0xc8(%rdi), %rcx\n"
#define RIP_M "_g_pages+0x1100(%rip)"

#define RUN_POST32 "fldl 0x18(%eax)\nfmulp\nfadd %st(1), %st\n"
#define RUN_POST_POP32 "fldl 0x18(%eax)\nfmul %st(1), %st\nfadd %st(1), %st\n"
#define TR_PRE32 "fildll 0x38(%eax)\nfilds 0xc0(%eax)\nfilds 0xc0(%eax)\n"
#define TR_POST32 "fistpl 8(%edx)\nfildl 0x24(%eax)\nfistpll 16(%edx)\n"

#define CASES(X)                                                                                  \
    /* alone in a reply */                                                                        \
    X(PRECISE, s_fld32, M_F32, IN, P_NONE, E_RD, "", "flds (%rsi)", "")                           \
    X(PRECISE, s_fld64, M_F64, IN, P_NONE, E_RD, "", "fldl (%rsi)", "")                           \
    X(PRECISE, s_fld80, M_F80, IN, P_NONE, E_RD, "", "fldt (%rsi)", "")                           \
    X(PRECISE, s_fst32, M_NONE, IN, P_NONE, E_WP, "", "fsts (%rsi)", "")                          \
    X(PRECISE, s_fst64, M_NONE, IN, P_NONE, E_WP, "", "fstl (%rsi)", "")                          \
    X(PRECISE, s_fstp32, M_NONE, IN, P_NONE, E_WP, "", "fstps (%rsi)", "")                        \
    X(PRECISE, s_fstp64, M_NONE, IN, P_NONE, E_WP, "", "fstpl (%rsi)", "")                        \
    X(PRECISE, s_fstp80, M_NONE, IN, P_NONE, E_WP, "", "fstpt (%rsi)", "")                        \
    X(PRECISE, s_fild16, M_I16, IN, P_NONE, E_RD, "", "filds (%rsi)", "")                         \
    X(PRECISE, s_fild32, M_I32, IN, P_NONE, E_RD, "", "fildl (%rsi)", "")                         \
    X(PRECISE, s_fild64, M_I64, IN, P_NONE, E_RD, "", "fildll (%rsi)", "")                        \
    X(PRECISE, s_fist16, M_NONE, IN, P_NONE, E_WR, "", "fists (%rsi)", "")                        \
    X(PRECISE, s_fist32, M_NONE, IN, P_NONE, E_WR, "", "fistl (%rsi)", "")                        \
    X(PRECISE, s_fistp16, M_NONE, IN, P_NONE, E_WR, "", "fistps (%rsi)", "")                      \
    X(PRECISE, s_fistp32, M_NONE, IN, P_NONE, E_WR, "", "fistpl (%rsi)", "")                      \
    X(PRECISE, s_fistp64, M_NONE, IN, P_NONE, E_WR, "", "fistpll (%rsi)", "")                     \
    X(PRECISE, s_fisttp32, M_NONE, IN, P_NONE, E_WR, "", "fisttpl (%rsi)", "")                    \
    X(PRECISE, s_fbld, M_BCD, IN, P_NONE, E_RD, "", "fbld (%rsi)", "")                            \
    X(PRECISE, s_fbstp, M_NONE, IN, P_NONE, E_WP, "", "fbstp (%rsi)", "")                         \
    X(PRECISE, s_fldcw, M_CW, IN, P_NONE, E_RD, "", "fldcw (%rsi)", "")                           \
    X(PRECISE, s_fnstcw, M_NONE, IN, P_NONE, E_WR, "", "fnstcw (%rsi)", "")                       \
    X(PRECISE, s_fnstsw, M_NONE, IN, P_NONE, E_WP, "", "fnstsw (%rsi)", "")                       \
    X(PRECISE, s_fnsave, M_NONE, IN, P_NONE, E_WP, "", "fnsave (%rsi)", "")                       \
    X(PRECISE, s_frstor, M_SAVE, IN, P_NONE, E_RD, "", "frstor (%rsi)", "")                       \
    X(PRECISE, s_fnstenv, M_NONE, IN, P_NONE, E_WR, "", "fnstenv (%rsi)", "")                     \
    X(PRECISE, s_fldenv, M_ENV, IN, P_NONE, E_RD, "", "fldenv (%rsi)", "")                        \
    X(PRECISE, s_fadd64, M_F64, IN, P_NONE, E_RD, "", "faddl (%rsi)", "")                         \
    X(PRECISE, s_fmul32, M_F32, IN, P_NONE, E_RD, "", "fmuls (%rsi)", "")                         \
    X(PRECISE, s_fsubr64, M_F64, IN, P_NONE, E_RD, "", "fsubrl (%rsi)", "")                       \
    X(PRECISE, s_fdiv32, M_F32, IN, P_NONE, E_RD, "", "fdivs (%rsi)", "")                         \
    X(PRECISE, s_fcom64, M_F64, IN, P_NONE, E_RD, "", "fcoml (%rsi)", "")                         \
    X(PRECISE, s_fcomp32, M_F32, IN, P_NONE, E_RD, "", "fcomps (%rsi)", "")                       \
    X(PRECISE, s_fiadd32, M_I32, IN, P_NONE, E_RD, "", "fiaddl (%rsi)", "")                       \
    X(PRECISE, s_fimul16, M_I16, IN, P_NONE, E_RD, "", "fimuls (%rsi)", "")                       \
    X(PRECISE, s_ficom32, M_I32, IN, P_NONE, E_RD, "", "ficoml (%rsi)", "")                       \
    /* first instruction of a longer run */                                                       \
    X(PRECISE, r_first_fld64, M_F64, IN, P_NONE, E_RD, "", "fldl (%rsi)", RUN_POST)               \
    X(PRECISE, r_first_fld80, M_F80, IN, P_NONE, E_RD, "", "fldt (%rsi)", RUN_POST)               \
    X(PRECISE, r_first_fild32, M_I32, IN, P_NONE, E_RD, "", "fildl (%rsi)", RUN_POST)             \
    X(PRECISE, r_first_fadd64, M_F64, IN, P_NONE, E_RD, "", "faddl (%rsi)", RUN_POST)             \
    X(PRECISE, r_first_fstp64, M_NONE, IN, P_NONE, E_WP, "", "fstpl (%rsi)", RUN_POST_POP)        \
    X(PRECISE, r_first_fistp32, M_NONE, IN, P_NONE, E_WP, "", "fistpl (%rsi)", RUN_POST_POP)      \
    X(PRECISE, r_first_fnstcw, M_NONE, IN, P_NONE, E_WP, "", "fnstcw (%rsi)", RUN_POST)           \
    X(PRECISE, b_first_fld64, M_F64, IN, P_NONE, E_RD, "", "fldl (%rsi)", BRIDGE_POST)            \
    X(PRECISE, b_first_fstp64, M_NONE, IN, P_NONE, E_WP, "", "fstpl (%rsi)",                      \
      "fldl 0x18(%rdi)\n" BRIDGE_POST)                                                            \
    /* first half of a two-instruction run that a fusion takes */                                 \
    X(PRECISE, f_fld_fstp_load, M_F64, IN, P_NONE, E_RD, "", "fldl (%rsi)", "fstpl (%rdx)")       \
    X(PRECISE, f_m80_load, M_F80, IN, P_NONE, E_RD, "", "fldt (%rsi)", "fstpt 0x20(%rdx)")        \
    X(PRECISE, f_arith_fstp_load, M_F64, IN, P_NONE, E_RD, "", "fmull (%rsi)", "fstpl (%rdx)")    \
    X(PRECISE, f_fstp_fld_store, M_NONE, IN, P_NONE, E_WP, "", "fstpl (%rsi)", "fldl 0x10(%rdi)") \
    X(PRECISE, f_fld_arithp, M_F64, IN, P_NONE, E_RD, "", "fldl (%rsi)", "faddp")                 \
    X(PRECISE, f_fld_fcomp, M_F64, IN, P_NONE, E_RD, "", "fldl (%rsi)", "fcompl 0x10(%rdi)")      \
    X(PRECISE, f_fld_arith_fstp_load, M_F64, IN, P_NONE, E_RD, "", "fldl (%rsi)",                 \
      "fmull 0x10(%rdi)\nfstpl (%rdx)")                                                           \
    /* integer loads and stores only */                                                           \
    X(PRECISE, t_fild16, M_I16, IN, P_NONE, E_RD, TR_PRE, "filds (%rsi)",                         \
      "fistps (%rdx)\n" TR_POST)                                                                  \
    X(PRECISE, t_fild32, M_I32, IN, P_NONE, E_RD, TR_PRE, "fildl (%rsi)",                         \
      "fistpl (%rdx)\n" TR_POST)                                                                  \
    X(PRECISE, t_fild64, M_I64, IN, P_NONE, E_RD, TR_PRE, "fildll (%rsi)",                        \
      "fistpll (%rdx)\n" TR_POST)                                                                 \
    X(PRECISE, t_fist32, M_NONE, IN, P_NONE, E_WR, TR_PRE, "fistl (%rsi)", TR_POST)               \
    X(PRECISE, t_fistp16, M_NONE, IN, P_NONE, E_WR, TR_PRE, "fistps (%rsi)",                      \
      "filds 0xc0(%rdi)\n" TR_POST)                                                               \
    X(PRECISE, t_fistp64, M_NONE, IN, P_NONE, E_WR, TR_PRE, "fistpll (%rsi)",                     \
      "filds 0xc0(%rdi)\n" TR_POST)                                                               \
    X(PRECISE, t_fisttp32, M_NONE, IN, P_NONE, E_WR, TR_PRE, "fisttpl (%rsi)",                    \
      "filds 0xc0(%rdi)\n" TR_POST)                                                               \
    X(PRECISE, t_fldcw, M_CW, IN, P_NONE, E_RD, TR_PRE, "fldcw (%rsi)", TR_POST)                  \
    X(PRECISE, t_fnstcw, M_NONE, IN, P_NONE, E_WR, TR_PRE, "fnstcw (%rsi)", TR_POST)              \
    X(PRECISE, t_first_fild64, M_I64, IN, P_NONE, E_RD, "", "fildll (%rsi)",                      \
      "fistpll (%rdx)\n" TR_POST)                                                                 \
    X(PRECISE, t_pair_load, M_I64, IN, P_NONE, E_RD, "", "fildll (%rsi)", "fistpll (%rdx)")       \
    X(PRECISE, t_nested_store, M_NONE, IN, P_NONE, E_WR,                                          \
      "fildll 0x38(%rdi)\nfildll 0x38(%rdi)\nfistpll 8(%rdx)\n", "fistpll (%rsi)", "")            \
    /* the access straddles into the protected page */                                            \
    X(PRECISE, x_fld64, M_F64, CROSS, P_NONE, E_RD, "", "fldl (%rsi)", "")                        \
    X(PRECISE, x_fld80, M_F80, CROSS, P_NONE, E_RD, "", "fldt (%rsi)", RUN_POST)                  \
    X(PRECISE, x_fstp64, M_NONE, CROSS, P_NONE, E_WP, "", "fstpl (%rsi)", "")                     \
    X(PRECISE, x_fistp64, M_NONE, CROSS, P_NONE, E_WR, TR_PRE, "fistpll (%rsi)", TR_POST)         \
    X(PRECISE, x_fnsave, M_NONE, CROSS, P_NONE, E_WP, "", "fnsave (%rsi)", "")                    \
    /* stores into a read-only page */                                                            \
    X(PRECISE, o_fstp32, M_NONE, IN, P_RO, E_WR, "", "fstps (%rsi)", "")                          \
    X(PRECISE, o_fst64, M_NONE, IN, P_RO, E_WR, "", "fstl (%rsi)", "")                            \
    X(PRECISE, o_fstp80, M_NONE, IN, P_RO, E_WR, "", "fstpt (%rsi)", "")                          \
    X(PRECISE, o_fistp32, M_NONE, IN, P_RO, E_WR, "", "fistpl (%rsi)", "")                        \
    X(PRECISE, o_fistp64_tr, M_NONE, IN, P_RO, E_WR, TR_PRE, "fistpll (%rsi)", TR_POST)           \
    X(PRECISE, o_fnstcw, M_NONE, IN, P_RO, E_WR, "", "fnstcw (%rsi)", "")                         \
    X(PRECISE, o_fnstsw, M_NONE, IN, P_RO, E_WR, "", "fnstsw (%rsi)", "")                         \
    X(PRECISE, o_fnsave, M_NONE, IN, P_RO, E_WR, "", "fnsave (%rsi)", "")                         \
    X(PRECISE, o_fbstp, M_NONE, IN, P_RO, E_WR, "", "fbstp (%rsi)", "")                           \
    X(PRECISE, o_first_fstp64, M_NONE, IN, P_RO, E_WR, "", "fstpl (%rsi)", RUN_POST_POP)          \
    X(PRECISE, o_fstp_fld, M_NONE, IN, P_RO, E_WR, "", "fstpl (%rsi)", "fldl 0x10(%rdi)")         \
    X(PRECISE, o_cross_fstp64, M_NONE, CROSS, P_RO, E_WR, "", "fstpl (%rsi)", "")                 \
    X(PRECISE, o_cross_fstp80, M_NONE, CROSS, P_RO, E_WR, "", "fstpt (%rsi)", "")                 \
    /* other operand forms */                                                                     \
    X(PRECISE, a_disp_fld64, M_F64, IN, P_NONE, E_RD, DISP_PRE, "fldl 0x40(%r8)", "")             \
    X(PRECISE, a_disp_fstp64, M_NONE, IN, P_NONE, E_WP, DISP_PRE, "fstpl 0x40(%r8)", "")          \
    X(PRECISE, a_disp_fadd64_run, M_F64, IN, P_NONE, E_RD, DISP_PRE, "faddl 0x40(%r8)", RUN_POST) \
    X(PRECISE, a_index_fld64, M_F64, IN, P_NONE, E_RD, INDEX_PRE, "fldl 0x8(%r8,%rcx,4)", "")     \
    X(PRECISE, a_index_fstp64, M_NONE, IN, P_NONE, E_WP, INDEX_PRE, "fstpl 0x8(%r8,%rcx,4)", "")  \
    X(PRECISE, a_index_fistp64_tr, M_NONE, IN, P_NONE, E_WR, TR_PRE INDEX_PRE,                    \
      "fistpll 0x8(%r8,%rcx,4)", TR_POST)                                                         \
    X(PRECISE, a_rip_fld64, M_F64, IN, P_NONE, E_RD, "", "fldl " RIP_M, "")                       \
    X(PRECISE, a_rip_fstp64, M_NONE, IN, P_NONE, E_WP, "", "fstpl " RIP_M, "")                    \
    X(PRECISE, a_rip_fstp64_ro, M_NONE, IN, P_RO, E_WR, "", "fstpl " RIP_M, "")                   \
    X(PRECISE, a_gs_fld64, M_F64, IN, P_NONE, E_RD, GS_PRE, "fldl %gs:(%rcx)", "")                \
    X(PRECISE, a_gs_fstp64, M_NONE, IN, P_NONE, E_WP, GS_PRE, "fstpl %gs:(%rcx)", "")             \
    X(PRECISE, a_gs_fstp64_ro, M_NONE, IN, P_RO, E_WR, GS_PRE, "fstpl %gs:(%rcx)", "")            \
    /* later in a floating-point run: precise only with X87_NATIVE_SLOTS=1 */                     \
    X(IN_RUN, r_fld32, M_F32, IN, P_NONE, E_RD, RUN_PRE, "flds (%rsi)", RUN_POST)                 \
    X(IN_RUN, r_fld64, M_F64, IN, P_NONE, E_RD, RUN_PRE, "fldl (%rsi)", RUN_POST)                 \
    X(IN_RUN, r_fld80, M_F80, IN, P_NONE, E_RD, RUN_PRE, "fldt (%rsi)", RUN_POST)                 \
    X(IN_RUN, r_fst32, M_NONE, IN, P_NONE, E_WP, RUN_PRE, "fsts (%rsi)", RUN_POST)                \
    X(IN_RUN, r_fst64, M_NONE, IN, P_NONE, E_WP, RUN_PRE, "fstl (%rsi)", RUN_POST)                \
    X(IN_RUN, r_fstp32, M_NONE, IN, P_NONE, E_WP, RUN_PRE, "fstps (%rsi)", RUN_POST)              \
    X(IN_RUN, r_fstp64, M_NONE, IN, P_NONE, E_WP, RUN_PRE, "fstpl (%rsi)", RUN_POST)              \
    X(IN_RUN, r_fstp80, M_NONE, IN, P_NONE, E_WP, RUN_PRE, "fstpt (%rsi)", RUN_POST)              \
    X(IN_RUN, r_fild16, M_I16, IN, P_NONE, E_RD, RUN_PRE, "filds (%rsi)", RUN_POST)               \
    X(IN_RUN, r_fild32, M_I32, IN, P_NONE, E_RD, RUN_PRE, "fildl (%rsi)", RUN_POST)               \
    X(IN_RUN, r_fild64, M_I64, IN, P_NONE, E_RD, RUN_PRE, "fildll (%rsi)", RUN_POST)              \
    X(IN_RUN, r_fist16, M_NONE, IN, P_NONE, E_WP, RUN_PRE, "fists (%rsi)", RUN_POST)              \
    X(IN_RUN, r_fist32, M_NONE, IN, P_NONE, E_WP, RUN_PRE, "fistl (%rsi)", RUN_POST)              \
    X(IN_RUN, r_fistp32, M_NONE, IN, P_NONE, E_WP, RUN_PRE, "fistpl (%rsi)", RUN_POST)            \
    X(IN_RUN, r_fistp64, M_NONE, IN, P_NONE, E_WP, RUN_PRE, "fistpll (%rsi)", RUN_POST)           \
    X(IN_RUN, r_fisttp32, M_NONE, IN, P_NONE, E_WP, RUN_PRE, "fisttpl (%rsi)", RUN_POST)          \
    X(IN_RUN, r_fbld, M_BCD, IN, P_NONE, E_RD, RUN_PRE, "fbld (%rsi)", RUN_POST)                  \
    X(IN_RUN, r_fbstp, M_NONE, IN, P_NONE, E_WP, RUN_PRE, "fbstp (%rsi)", RUN_POST)               \
    X(IN_RUN, r_fldcw, M_CW, IN, P_NONE, E_RD, RUN_PRE, "fldcw (%rsi)", RUN_POST)                 \
    X(IN_RUN, r_fnstcw, M_NONE, IN, P_NONE, E_WP, RUN_PRE, "fnstcw (%rsi)", RUN_POST)             \
    X(IN_RUN, r_fnstsw, M_NONE, IN, P_NONE, E_WP, RUN_PRE, "fnstsw (%rsi)", RUN_POST)             \
    X(IN_RUN, r_fnsave, M_NONE, IN, P_NONE, E_WP, RUN_PRE, "fnsave (%rsi)", RUN_POST)             \
    X(IN_RUN, r_frstor, M_SAVE, IN, P_NONE, E_RD, RUN_PRE, "frstor (%rsi)", RUN_POST)             \
    X(IN_RUN, r_fadd64, M_F64, IN, P_NONE, E_RD, RUN_PRE, "faddl (%rsi)", RUN_POST)               \
    X(IN_RUN, r_fmul32, M_F32, IN, P_NONE, E_RD, RUN_PRE, "fmuls (%rsi)", RUN_POST)               \
    X(IN_RUN, r_fsubr64, M_F64, IN, P_NONE, E_RD, RUN_PRE, "fsubrl (%rsi)", RUN_POST)             \
    X(IN_RUN, r_fdiv32, M_F32, IN, P_NONE, E_RD, RUN_PRE, "fdivs (%rsi)", RUN_POST)               \
    X(IN_RUN, r_fcom64, M_F64, IN, P_NONE, E_RD, RUN_PRE, "fcoml (%rsi)", RUN_POST)               \
    X(IN_RUN, r_fcomp32, M_F32, IN, P_NONE, E_RD, RUN_PRE, "fcomps (%rsi)", RUN_POST)             \
    X(IN_RUN, r_fiadd32, M_I32, IN, P_NONE, E_RD, RUN_PRE, "fiaddl (%rsi)", RUN_POST)             \
    X(IN_RUN, r_fimul16, M_I16, IN, P_NONE, E_RD, RUN_PRE, "fimuls (%rsi)", RUN_POST)             \
    X(IN_RUN, r_ficom32, M_I32, IN, P_NONE, E_RD, RUN_PRE, "ficoml (%rsi)", RUN_POST)             \
    X(IN_RUN, r_last_fld64, M_F64, IN, P_NONE, E_RD, RUN_PRE "fldl 0x18(%rdi)\nfaddp\n",          \
      "fldl (%rsi)", "")                                                                          \
    X(IN_RUN, r_last_fstp64, M_NONE, IN, P_NONE, E_WP, RUN_PRE "fldl 0x18(%rdi)\nfaddp\n",        \
      "fstpl (%rsi)", "")                                                                         \
    X(IN_RUN, b_mid_fld64, M_F64, IN, P_NONE, E_RD, "fldl 0x10(%rdi)\nmov 0x38(%rdi), %rcx\n",    \
      "fldl (%rsi)", "faddp\nfstpl (%rdx)")                                                       \
    X(IN_RUN, f_fld_fstp_store, M_NONE, IN, P_NONE, E_WP, "fldl 0x10(%rdi)\n", "fstpl (%rsi)",    \
      "")                                                                                         \
    X(IN_RUN, f_fld_fstp32_store, M_NONE, IN, P_NONE, E_WP, "fldl 0x18(%rdi)\n", "fstps (%rsi)",  \
      "")                                                                                         \
    X(IN_RUN, f_m80_store, M_NONE, IN, P_NONE, E_WP, "fldt 0x40(%rdi)\n", "fstpt (%rsi)", "")     \
    X(IN_RUN, f_arith_fstp_store, M_NONE, IN, P_NONE, E_WP, "fmull 0x10(%rdi)\n",                 \
      "fstpl (%rsi)", "")                                                                         \
    X(IN_RUN, f_fstp_fld_load, M_F64, IN, P_NONE, E_RD, "fstpl (%rdx)\n", "fldl (%rsi)", "")      \
    X(IN_RUN, f_fld_arith_fstp_mid, M_F64, IN, P_NONE, E_RD, "fldl 0x10(%rdi)\n", "fmull (%rsi)", \
      "fstpl (%rdx)")                                                                             \
    X(IN_RUN, f_fld_arith_fstp_store, M_NONE, IN, P_NONE, E_WP,                                   \
      "fldl 0x10(%rdi)\nfmull 0x18(%rdi)\n", "fstpl (%rsi)", "")                                  \
    X(IN_RUN, f_arithp_fstp, M_NONE, IN, P_NONE, E_WP, "faddp\n", "fstpl (%rsi)", "")             \
    /* the store of a fused fild m64 / fistp m64 pair: rolled back to the fild */                 \
    X(IN_RUN, t_pair_store, M_NONE, IN, P_NONE, E_WR, "fildll 0x38(%rdi)\n", "fistpll (%rsi)", "")

/* 32-bit compatibility mode. */
#define CASES32(X)                                                                                \
    X(PRECISE, c_fld64, M_F64, IN, P_NONE, E_RD, "", "fldl (%ebx)", "")                           \
    X(PRECISE, c_fstp64, M_NONE, IN, P_NONE, E_WP, "", "fstpl (%ebx)", "")                        \
    X(PRECISE, c_fstp80, M_NONE, IN, P_NONE, E_WP, "", "fstpt (%ebx)", "")                        \
    X(PRECISE, c_fistp32, M_NONE, IN, P_NONE, E_WR, "", "fistpl (%ebx)", "")                      \
    X(PRECISE, c_fild64_tr, M_I64, IN, P_NONE, E_RD, TR_PRE32, "fildll (%ebx)",                   \
      "fistpll (%edx)\n" TR_POST32)                                                               \
    X(PRECISE, c_first_fadd64, M_F64, IN, P_NONE, E_RD, "", "faddl (%ebx)", RUN_POST32)           \
    X(PRECISE, c_first_fstp64, M_NONE, IN, P_NONE, E_WP, "", "fstpl (%ebx)", RUN_POST_POP32)      \
    X(PRECISE, c_fnstcw, M_NONE, IN, P_NONE, E_WR, "", "fnstcw (%ebx)", "")                       \
    X(PRECISE, c_fstp64_ro, M_NONE, IN, P_RO, E_WR, "", "fstpl (%ebx)", "")                       \
    X(PRECISE, c_fistp64_ro, M_NONE, IN, P_RO, E_WR, "", "fistpll (%ebx)", "")                    \
    X(PRECISE, c_disp_fstp64, M_NONE, IN, P_NONE, E_WP, "lea -0x40(%ebx), %ecx\n",                \
      "fstpl 0x40(%ecx)", "")                                                                     \
    X(PRECISE, c_index_fld64, M_F64, IN, P_NONE, E_RD, "mov $0x10, %edx\nlea -0x48(%ebx), %ecx\n",\
      "fldl 0x8(%ecx,%edx,4)", "")                                                                \
    X(IN_RUN, c_r_fstp64, M_NONE, IN, P_NONE, E_WP, "fldl 0x10(%eax)\nfmul %st(1), %st\n",        \
      "fstpl (%ebx)", RUN_POST32)

CASES(DEFINE_CASE)
CASES32(DEFINE_CASE32)

struct test_case {
    enum group group;
    const char* name;
    enum init init;
    int off;
    enum prot prot;
    enum err err;
    int compat;
    const char* fn;
    const char* ref;
    const char* skip;
    const char* fault;
    const char* after;
};

#define TABLE_ENTRY(group, name, init, off, prot, err, pre, insn, post) \
    {group, #name, init, off, prot, err, 0, name, name##_ref, name##_skip, name##_fault, name##_after},
#define TABLE_ENTRY32(group, name, init, off, prot, err, pre, insn, post) \
    {group, #name, init, off, prot, err, 1, name, name##_ref, name##_skip, name##_fault, name##_after},
static const struct test_case cases[] = {CASES(TABLE_ENTRY) CASES32(TABLE_ENTRY32)};

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
enum mode { MODE_NONE, MODE_RETRY, MODE_SKIP, MODE_WC };

static char* page;
static char* scratch;
static volatile enum mode mode;
static const char* volatile skip_to;
static volatile int faults;
static struct {
    uint64_t rip;
    uint64_t addr;
    uint32_t err, trapno;
    int sig;
    uint16_t fcw, fsw;
    uint8_t ftw;
    uint8_t st[8][10];
} seen;
static int failures;
static sigjmp_buf wc_jmp;
static volatile int wc_sig;

/* Rosetta uses the full mcontext once an LDT is installed.  Its extra
 * segment registers move the FPU area; the exception state comes first and
 * the 64-bit thread state leads the full thread state, so __es and rip stay
 * where they are. */
static _STRUCT_X86_FLOAT_STATE64* float_state(ucontext_t* uc) {
    if (uc->uc_mcsize == sizeof(_STRUCT_MCONTEXT64_FULL) ||
        uc->uc_mcsize == sizeof(_STRUCT_MCONTEXT_AVX64_FULL) ||
        uc->uc_mcsize == sizeof(_STRUCT_MCONTEXT_AVX512_64_FULL))
        return &((_STRUCT_MCONTEXT64_FULL*)uc->uc_mcontext)->__fs;
    return &uc->uc_mcontext->__fs;
}

static void handler(int sig, siginfo_t* info, void* context) {
    ucontext_t* uc = context;
    if (mode == MODE_WC) {
        wc_sig = sig;
        siglongjmp(wc_jmp, 1);
    }
    _STRUCT_X86_FLOAT_STATE64* fs = float_state(uc);
    faults++;
    if (faults == 1) {
        seen.sig = sig;
        seen.rip = uc->uc_mcontext->__ss.__rip;
        seen.addr = (uint64_t)info->si_addr;
        seen.err = uc->uc_mcontext->__es.__err;
        seen.trapno = uc->uc_mcontext->__es.__trapno;
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

/* ── 32-bit compatibility mode, as in test_x87_signal_context ──────────── */

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
        printf("FAIL  sync fault: i386_set_ldt(%d) failed (errno=%d %s)\n", idx, errno,
               strerror(errno));
        failures++;
        return 0;
    }
    return (uint16_t)((idx << 3) | (1 << 2) | 3);
}

struct __attribute__((packed)) farptr32 {
    uint32_t off;
    uint16_t sel;
};

static void call32(const char* entry, struct io* io) {
    struct farptr32 to32 = {(uint32_t)(uintptr_t)entry, sel_cs32};
    uint64_t in_eax = (uintptr_t)io;
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

static void invoke(const struct test_case* c, const char* entry, struct io* io) {
    if (c->compat)
        call32(entry, io);
    else
        ((void (*)(struct io*))(uintptr_t)entry)(io);
}

/* ── %gs ───────────────────────────────────────────────────────────────── */

/* The %gs base is the thread's TSD array: find it from a slot we set. */
static uintptr_t gs_base;

static int find_gs_base(void) {
    static char marker;
    pthread_key_t key;
    if (pthread_key_create(&key, NULL) != 0 || pthread_setspecific(key, &marker) != 0)
        return 0;
    const uintptr_t slot = (uintptr_t)key * 8;
    uint64_t via_gs;
    __asm__ volatile("movq %%gs:(%1), %0" : "=r"(via_gs) : "r"(slot));
    if (via_gs != (uintptr_t)&marker)
        return 0;
    const char* self = (const char*)pthread_self();
    for (size_t off = 0; off < 16384; off += 8) {
        void* v;
        memcpy(&v, self + off, 8);
        if (v == &marker && off >= slot) {
            gs_base = (uintptr_t)(self + off) - slot;
            /* Check by reading the protected page's neighbour through %gs. */
            uint64_t want = 0x6773626173652121ULL, got;
            memcpy(g_pages + 8, &want, 8);
            __asm__ volatile("movq %%gs:(%1), %0"
                             : "=r"(got)
                             : "r"((uintptr_t)(g_pages + 8) - gs_base));
            return got == want;
        }
    }
    return 0;
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
    io->gsoff = (int64_t)((uintptr_t)io->target - gs_base);
    memset(scratch, 0xa5, SCRATCH);
    mprotect(page, 4096, PROT_READ | PROT_WRITE);
    memset(page - 4096, 0x5a, 8192);
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

/* Status word exception flags, stack fault and summary bit.  Stock Rosetta
 * sets them and the sidecar does not (docs/internals.md), and under
 * allocation failure injection the same instruction can be translated by
 * either one from run to run, so they are not compared. */
static const uint16_t kExceptionFlags = 0x00ff;

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
    const uint32_t want_err = c->err == E_RD ? 0x4 : 0x6;
    const int err_ok = seen.err == want_err ||
                       (c->err == E_WP && c->prot == P_NONE && seen.err == 0x4);
    if (!err_ok || seen.trapno != 14) {
        printf("FAIL  sync fault %s %s: fault kind err=%#x trapno=%u, expected err=%#x%s "
               "trapno=14\n",
               c->name, how, seen.err, seen.trapno, want_err,
               c->err == E_WP && c->prot == P_NONE ? " or 0x4" : "");
        bad++;
    }
    if (seen.fcw != fcw) {
        printf("FAIL  sync fault %s %s: control word %04x, expected %04x\n", c->name, how,
               seen.fcw, fcw);
        bad++;
    }
    if ((seen.fsw & ~kExceptionFlags) != (fsw & ~kExceptionFlags)) {
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
    if (u16(got->save) != u16(want->save) ||
        (u16(got->save + 4) & ~kExceptionFlags) != (u16(want->save + 4) & ~kExceptionFlags) ||
        u16(got->save + 8) != u16(want->save + 8) || regs_differ) {
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
    invoke(c, c->fn, &io);
    capture(&normal, &io);

    prepare(&io, c);
    invoke(c, c->ref, &io);
    memcpy(ref, io.save, 108);

    prepare(&io, c);
    capture(&untouched, &io);
    invoke(c, c->skip, &io);
    capture(&skipped, &io);
    memcpy(skipped.region, untouched.region, REGION);
    if (faults != 0) {
        printf("FAIL  sync fault %s: %d faults without a protected page\n", c->name, faults);
        bad++;
    }

    static const enum mode modes[] = {MODE_RETRY, MODE_SKIP};
    uint32_t err = 0;
    for (int m = 0; m < 2; m++) {
        const char* how = modes[m] == MODE_RETRY ? "retry" : "skip";
        prepare(&io, c);
        mprotect(page, 4096, c->prot == P_RO ? PROT_READ : PROT_NONE);
        memset(&seen, 0, sizeof(seen));
        faults = 0;
        skip_to = c->after;
        mode = modes[m];
        invoke(c, c->fn, &io);
        mode = MODE_NONE;
        capture(&got, &io);
        if (faults != 1) {
            printf("FAIL  sync fault %s %s: %d faults, expected 1\n", c->name, how, faults);
            bad++;
            if (faults == 0)
                continue;
        }
        err = seen.err;
        bad += check_context(c, how, ref, (uintptr_t)io.target);
        bad += compare_outcome(c, how, &got, modes[m] == MODE_RETRY ? &normal : &skipped);
    }
    if (!bad)
        printf("PASS  sync fault %s (signal %d, err %#x)\n", c->name, seen.sig, err);
    failures += bad;
}

/* ── write-combined and uncached memory ────────────────────────────────── */

/* x87 stores into the buffer: the first instruction of a reply (probed),
 * one inside a run, an integer store, a 10-byte store, and loads.  All
 * operands are naturally aligned, which uncached memory requires of every
 * access. */
__asm__(".text\n.p2align 4\n_wc_x87:\n"
        "fninit\nfld1\nlfence\n"
        "fstps (%rdi)\nlfence\n"
        "fld1\nlfence\nfstpl 8(%rdi)\nlfence\n"
        "fld1\nlfence\nfstpt 16(%rdi)\nlfence\n"
        "fld1\nlfence\nfistpll 32(%rdi)\nlfence\n"
        "fld1\nfld1\nfaddp\nfstpl 40(%rdi)\nlfence\n"
        "fld1\nlfence\nfstpl 48(%rdi)\nfld1\nfstpl 56(%rdi)\nlfence\n"
        "flds (%rdi)\nlfence\nfaddl 8(%rdi)\nlfence\nfstpl 64(%rdi)\nlfence\n"
        "ret\n");
extern void wc_x87(void* p);

static void wc_case(const char* what, int cache_mode) {
    CFMutableDictionaryRef d = CFDictionaryCreateMutable(
        NULL, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    const int values[] = {256, 256, 4, 0x42475241, cache_mode};
    const CFStringRef keys[] = {kIOSurfaceWidth, kIOSurfaceHeight, kIOSurfaceBytesPerElement,
                                kIOSurfacePixelFormat, kIOSurfaceCacheMode};
    for (int i = 0; i < 5; i++) {
        CFNumberRef n = CFNumberCreate(NULL, kCFNumberIntType, &values[i]);
        CFDictionarySetValue(d, keys[i], n);
        CFRelease(n);
    }
    IOSurfaceRef s = IOSurfaceCreate(d);
    CFRelease(d);
    if (s == NULL) {
        printf("SKIP  sync fault wc %s: IOSurfaceCreate failed\n", what);
        return;
    }
    IOSurfaceLock(s, 0, NULL);
    /* Byte by byte: a memset or memcpy of uncached memory can use cache
     * maintenance or vector accesses that fault there. */
    volatile unsigned char* p = IOSurfaceGetBaseAddress(s);
    for (int i = 0; i < 128; i++)
        p[i] = 0;
    wc_sig = 0;
    mode = MODE_WC;
    if (sigsetjmp(wc_jmp, 1) == 0)
        wc_x87((void*)p);
    mode = MODE_NONE;
    unsigned char got[128];
    for (int i = 0; i < 128; i++)
        got[i] = p[i];
    IOSurfaceUnlock(s, 0, NULL);
    CFRelease(s);
    if (wc_sig != 0) {
        printf("FAIL  sync fault wc %s: signal %d\n", what, wc_sig);
        failures++;
        return;
    }
    float f;
    double d1, d2, d3, d4, d5;
    int64_t i;
    uint64_t m80;
    uint16_t e80;
    memcpy(&f, got, 4);
    memcpy(&d1, got + 8, 8);
    memcpy(&m80, got + 16, 8);
    memcpy(&e80, got + 24, 2);
    memcpy(&i, got + 32, 8);
    memcpy(&d2, got + 40, 8);
    memcpy(&d3, got + 48, 8);
    memcpy(&d4, got + 56, 8);
    memcpy(&d5, got + 64, 8);
    if (f != 1.0f || d1 != 1.0 || m80 != 0x8000000000000000ULL || e80 != 0x3fff || i != 1 ||
        d2 != 2.0 || d3 != 1.0 || d4 != 1.0 || d5 != 2.0) {
        printf("FAIL  sync fault wc %s: stored values differ (%g %g %016llx/%04x %lld %g %g %g "
               "%g)\n",
               what, f, d1, (unsigned long long)m80, e80, (long long)i, d2, d3, d4, d5);
        failures++;
        return;
    }
    printf("PASS  sync fault wc %s\n", what);
}

int main(void) {
    setvbuf(stdout, NULL, _IOLBF, 0);
    const size_t n = sizeof(cases) / sizeof(cases[0]);
    if (getenv("SYNC_FAULT_LIST")) {
        for (size_t i = 0; i < n; i++)
            printf("%s %s\n", cases[i].name, cases[i].group == PRECISE ? "precise" : "in-run");
        printf("wc precise\n");
        return 0;
    }
    page = g_pages + 4096;
    scratch = scratch_area;
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = handler;
    sa.sa_flags = SA_SIGINFO;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGSEGV, &sa, NULL) != 0 || sigaction(SIGBUS, &sa, NULL) != 0) {
        printf("FAIL  sync fault: sigaction\n");
        return 1;
    }
    sel_cs32 = alloc_ldt_seg(3, 1);
    sel_ds32 = alloc_ldt_seg(4, 0);
    __asm__ volatile("mov %%cs, %0" : "=r"(sel_cs64));
    if (!find_gs_base()) {
        printf("FAIL  sync fault: the %%gs base was not found\n");
        failures++;
    }
    if (failures)
        return 1;
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
    if (!only || strcmp(only, "wc") == 0) {
        wc_case("write-combined IOSurface", 0x400);  // kIOMapWriteCombineCache
        wc_case("uncached IOSurface", 0x100);        // kIOMapInhibitCache
        ran++;
    }
    if (!ran) {
        printf("FAIL  sync fault: no case named %s\n", only ? only : "");
        return 1;
    }
    printf("%d failure%s\n", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
