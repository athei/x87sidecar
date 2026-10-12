// test_x87_gs_operand.c -- x87 memory operands with a %gs: or %fs: override
//
// A GS-relative address takes more temporaries than a plain one: the
// address without the segment, a register for the segment base, and X29,
// which the TLS base routine requires.  Translators that hold their own
// temporaries before computing the address can run out of registers or
// find X29 taken, and an assert then kills the process.
//
// Every case below runs the same instruction sequence three times: with
// its operand at %gs:disp(%rcx), at %fs:disp(%rcx), and as a plain
// disp(%rcx) operand pointing at an ordinary buffer holding the same bytes.
// The segment runs are compared with the plain run bit for bit: the stored
// values, the memory operand's bytes afterwards, and the status word.  A
// representative subset is also checked against fixed expected values, so
// a fault shared by all three runs still fails.  Every run checks that the
// address and data registers (%rcx, %rdx, %rsi, %rdi) survive.  Cases
// cover each x87 memory form isolated between integer instructions
// (translated on its own), inside a run of x87 instructions (register
// cache live), and in the shapes the peephole fusions match.
//
// The GS operands address thread-specific-data slots: on macOS x86_64 the
// GS base is the thread's TSD array, so slot k is at %gs:(k * 8).  The test
// allocates consecutive pthread keys and uses their slots as the area.
// The FS base of a macOS x86_64 thread under Rosetta is zero, so the FS
// operands address an ordinary buffer.  FS takes one register fewer than
// GS and no fixed one; the FS runs are skipped, not failed, if the FS base
// turns out not to be zero.
//
// Compile:
//   clang -arch x86_64 -O2 -o test_x87_gs_operand test_x87_gs_operand.c

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// A flag-producing integer instruction ends an x87 run and is never
// folded into one.
#define SEP "cmpq %%r11, %%r11\n\t"

enum { kAreaQwords = 18, kAreaBytes = kAreaQwords * 8, kOutBytes = 64 };

// %rcx = area - 8, so 8(%rcx) is the first area byte; the displacement
// makes the address computation allocate a register of its own.
// %rdx = area, for the bare-register form.
#define GS_M "%%gs:8(%%rcx)"
#define GS_M4 "%%gs:12(%%rcx)"
#define GS_M8 "%%gs:16(%%rcx)"
#define GS_B "%%gs:(%%rdx)"
#define FS_M "%%fs:8(%%rcx)"
#define FS_M4 "%%fs:12(%%rcx)"
#define FS_M8 "%%fs:16(%%rcx)"
#define FS_B "%%fs:(%%rdx)"
#define PL_M "8(%%rcx)"
#define PL_M4 "12(%%rcx)"
#define PL_M8 "16(%%rcx)"
#define PL_B "(%%rdx)"

// ── Body shapes ────────────────────────────────────────────────────────────
//
// Each takes the opcode and the operand spellings.  Inputs are the doubles
// at (%rsi); results go to (%rdi).

// Loads.
#define LD_S(op, M, M4, M8, B) op " " M "\n\t" SEP "fstpl (%%rdi)\n\t"
#define LD_R(op, M, M4, M8, B)                                                           \
    "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" op " " M "\n\t" "fstpl (%%rdi)\n\t"            \
    "fstpl 8(%%rdi)\n\t" "fstpl 16(%%rdi)\n\t"
#define LD_B(op, M, M4, M8, B) "fldl (%%rsi)\n\t" op " " B "\n\t" "faddp\n\t" "fstpl (%%rdi)\n\t"

// ST(0) op= mem.
#define AR_S(op, M, M4, M8, B) "fldl (%%rsi)\n\t" SEP op " " M "\n\t" SEP "fstpl (%%rdi)\n\t"
#define AR_R(op, M, M4, M8, B)                                                           \
    "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" op " " M "\n\t" "fstpl (%%rdi)\n\t"            \
    "fstpl 8(%%rdi)\n\t"
#define AR_B(op, M, M4, M8, B) "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" op " " B "\n\t"     \
    "fstpl (%%rdi)\n\t" "fstpl 8(%%rdi)\n\t"

// Compares, with the status word saved straight after.
#define CM_S(op, M, M4, M8, B)                                                           \
    "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" SEP op " " M "\n\t" SEP "fnstsw %%ax\n\t"       \
    "movw %%ax, 48(%%rdi)\n\t"
#define CM_R(op, M, M4, M8, B)                                                           \
    "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" op " " M "\n\t" "fnstsw %%ax\n\t"              \
    "fstpl (%%rdi)\n\t" "movw %%ax, 48(%%rdi)\n\t"

// Stores; the _P shapes are for the popping forms.
#define STO_S(op, M, M4, M8, B) "fldl (%%rsi)\n\t" SEP op " " M "\n\t" SEP
#define STO_R(op, M, M4, M8, B)                                                          \
    "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" op " " M "\n\t" "fstpl (%%rdi)\n\t"            \
    "fstpl 8(%%rdi)\n\t"
#define STO_R_P(op, M, M4, M8, B)                                                        \
    "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" op " " M "\n\t" "fstpl (%%rdi)\n\t"
#define STO_B_P(op, M, M4, M8, B)                                                        \
    "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" op " " B "\n\t" "fstpl (%%rdi)\n\t"

// Control and status word.
#define CW_S(op, M, M4, M8, B)                                                           \
    "fldcw " M "\n\t" SEP "fldl 8(%%rsi)\n\t" SEP "fistpl 32(%%rdi)\n\t"
#define CW_R(op, M, M4, M8, B) "fldl 8(%%rsi)\n\t" "fldcw " M "\n\t" "fistpl 32(%%rdi)\n\t"
#define STCW_S(op, M, M4, M8, B) "fldl (%%rsi)\n\t" SEP "fnstcw " M "\n\t" SEP
#define STCW_R(op, M, M4, M8, B) "fldl (%%rsi)\n\t" "fnstcw " M "\n\t" "fstpl (%%rdi)\n\t"
#define STSW_S(op, M, M4, M8, B)                                                         \
    "fldl (%%rsi)\n\t" "fcoml 8(%%rsi)\n\t" SEP "fnstsw " M "\n\t" SEP
#define STSW_R(op, M, M4, M8, B)                                                         \
    "fldl (%%rsi)\n\t" "fcoml 8(%%rsi)\n\t" "fnstsw " M "\n\t" "fstpl (%%rdi)\n\t"

// FNSAVE + FRSTOR and FNSTENV + FLDENV on the same image.
#define SAVE_S(op, M, M4, M8, B)                                                         \
    "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" SEP "fnsave " M "\n\t" SEP "frstor " M "\n\t"  \
    SEP "fstpl (%%rdi)\n\t" "fstpl 8(%%rdi)\n\t"
#define SAVE_R(op, M, M4, M8, B)                                                         \
    "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" "fnsave " M "\n\t" "frstor " M "\n\t"          \
    "fstpl (%%rdi)\n\t" "fstpl 8(%%rdi)\n\t"
#define SAVE_B(op, M, M4, M8, B)                                                         \
    "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" "fnsave " B "\n\t" "frstor " B "\n\t"          \
    "fstpl (%%rdi)\n\t" "fstpl 8(%%rdi)\n\t"
#define ENV_S(op, M, M4, M8, B)                                                          \
    "fldl (%%rsi)\n\t" SEP "fnstenv " M "\n\t" SEP "fldenv " M "\n\t" SEP                 \
    "fstpl (%%rdi)\n\t"

// Integer stores with the RC cache live: two rounding stores in one run.
#define RC_PAIR(op, M, M4, M8, B)                                                        \
    "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" "fistpl " M "\n\t" "fistpl " M4 "\n\t"
#define RC_PAIR_CW(op, M, M4, M8, B)                                                     \
    "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" "fnstcw " M8 "\n\t" "fistpl " M "\n\t"         \
    "fistpl " M4 "\n\t"

// Peephole fusion shapes.
#define F_FILD_ARITH(op, M, M4, M8, B) "fildl " M "\n\t" "fmull 8(%%rsi)\n\t" SEP "fstpl (%%rdi)\n\t"
#define F_FILD_ARITH_FSTP(op, M, M4, M8, B)                                              \
    "fildl " M "\n\t" "fmull 8(%%rsi)\n\t" "fstpl (%%rdi)\n\t"
#define F_FLD_FILD_ARITH(op, M, M4, M8, B)                                               \
    "fldl (%%rsi)\n\t" "fildl " M "\n\t" "fmull 8(%%rsi)\n\t" "fstpl (%%rdi)\n\t"         \
    "fstpl 8(%%rdi)\n\t"
#define F_FLD_ARITHP(op, M, M4, M8, B)                                                   \
    "fldl (%%rsi)\n\t" op " " M "\n\t" "fsubrp\n\t" "fstpl (%%rdi)\n\t"
#define F_FLD_ARITH_ARITHP(op, M, M4, M8, B)                                             \
    "fldl (%%rsi)\n\t" op " " M "\n\t" "fmull 8(%%rsi)\n\t" "faddp\n\t" "fstpl (%%rdi)\n\t"
#define F_FLD_ARITHM_ARITHP(op, M, M4, M8, B)                                            \
    "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" "fmull " M "\n\t" "faddp\n\t"                  \
    "fstpl (%%rdi)\n\t"
#define F_FLD_FSTP_FROM(op, M, M4, M8, B) op " " M "\n\t" "fstpl (%%rdi)\n\t"
#define F_FLD_FSTP_TO(op, M, M4, M8, B) "fldl (%%rsi)\n\t" op " " M "\n\t"
// The FLD starts its run: with a push still deferred from earlier in the
// same run, the FLD + FCOMP + FNSTSW fusion reports a stale TOP in AX, which
// is a separate fault from the one under test here.
#define F_FLD_FCOMP_FSTSW(op, M, M4, M8, B)                                              \
    "fldl 8(%%rsi)\n\t" SEP op " " M "\n\t" "fcompl (%%rsi)\n\t" "fnstsw %%ax\n\t"        \
    "movw %%ax, 48(%%rdi)\n\t" SEP "fstpl (%%rdi)\n\t"
#define F_FLD_FCOMPM_FSTSW(op, M, M4, M8, B)                                             \
    "fldl 8(%%rsi)\n\t" "fldl (%%rsi)\n\t" op " " M "\n\t" "fnstsw %%ax\n\t"              \
    "movw %%ax, 48(%%rdi)\n\t" "fstpl (%%rdi)\n\t"
#define F_FLD_FCOMP(op, M, M4, M8, B)                                                    \
    "fldl 8(%%rsi)\n\t" op " " M "\n\t" "fcompl (%%rsi)\n\t" SEP "fnstsw %%ax\n\t"        \
    "movw %%ax, 48(%%rdi)\n\t" "fstpl (%%rdi)\n\t"
#define F_FLD_FCOMPP_FSTSW(op, M, M4, M8, B)                                             \
    "fldl (%%rsi)\n\t" op " " M "\n\t" "fcompp\n\t" "fnstsw %%ax\n\t"                     \
    "movw %%ax, 48(%%rdi)\n\t"
#define F_FLD_FLD_FUCOMPP(op, M, M4, M8, B)                                              \
    op " " M "\n\t" "fldl (%%rsi)\n\t" "fucompp\n\t" "fnstsw %%ax\n\t"                    \
    "movw %%ax, 48(%%rdi)\n\t"
#define F_ARITHP_FSTP(op, M, M4, M8, B)                                                  \
    "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" "faddp\n\t" op " " M "\n\t"
#define F_ARITH_FSTP(op, M, M4, M8, B)                                                   \
    "fldl (%%rsi)\n\t" op " " M "\n\t" "fstpl (%%rdi)\n\t"
#define F_ARITH_FSTPM(op, M, M4, M8, B)                                                  \
    "fldl (%%rsi)\n\t" "fmull 8(%%rsi)\n\t" op " " M "\n\t"
#define F_FSTP_ARITH_FSTP(op, M, M4, M8, B)                                              \
    "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" "fstpl " M "\n\t" "faddl " M "\n\t"            \
    "fstpl " M8 "\n\t"
#define F_ARITH_FADDP(op, M, M4, M8, B)                                                  \
    "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" op " " M "\n\t" "faddp\n\t" "fstpl (%%rdi)\n\t"
#define F_FSTP_FLD(op, M, M4, M8, B)                                                     \
    "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" "fstpl " M "\n\t" op " " M8 "\n\t"             \
    "fstpl (%%rdi)\n\t" "fstpl 8(%%rdi)\n\t"
#define F_FXCH_FSTP(op, M, M4, M8, B)                                                    \
    "fldl (%%rsi)\n\t" "fldl 8(%%rsi)\n\t" "fxch\n\t" op " " M "\n\t" "fstpl (%%rdi)\n\t"
#define F_FLD_GROUP(op, M, M4, M8, B)                                                    \
    "fldl " M "\n\t" "fldl " M8 "\n\t" "faddp\n\t" "fstpl (%%rdi)\n\t"

// ── Case table ─────────────────────────────────────────────────────────────

enum InitKind { K_NONE, K_F32, K_F64, K_F80, K_I16, K_I32, K_I64, K_BCD, K_CW };

#define CASES(X)                                                                         \
    X(flds_S, K_F32, LD_S, "flds")                                                       \
    X(fldl_S, K_F64, LD_S, "fldl")                                                       \
    X(fldt_S, K_F80, LD_S, "fldt")                                                       \
    X(filds_S, K_I16, LD_S, "filds")                                                     \
    X(fildl_S, K_I32, LD_S, "fildl")                                                     \
    X(fildll_S, K_I64, LD_S, "fildll")                                                   \
    X(fbld_S, K_BCD, LD_S, "fbld")                                                       \
    X(flds_R, K_F32, LD_R, "flds")                                                       \
    X(fldl_R, K_F64, LD_R, "fldl")                                                       \
    X(fldt_R, K_F80, LD_R, "fldt")                                                       \
    X(filds_R, K_I16, LD_R, "filds")                                                     \
    X(fildl_R, K_I32, LD_R, "fildl")                                                     \
    X(fildll_R, K_I64, LD_R, "fildll")                                                   \
    X(fbld_R, K_BCD, LD_R, "fbld")                                                       \
    X(fldl_B, K_F64, LD_B, "fldl")                                                       \
    X(fildl_B, K_I32, LD_B, "fildl")                                                     \
    X(fadds_S, K_F32, AR_S, "fadds")                                                     \
    X(faddl_S, K_F64, AR_S, "faddl")                                                     \
    X(fsubs_S, K_F32, AR_S, "fsubs")                                                     \
    X(fsubl_S, K_F64, AR_S, "fsubl")                                                     \
    X(fsubrs_S, K_F32, AR_S, "fsubrs")                                                   \
    X(fsubrl_S, K_F64, AR_S, "fsubrl")                                                   \
    X(fmuls_S, K_F32, AR_S, "fmuls")                                                     \
    X(fmull_S, K_F64, AR_S, "fmull")                                                     \
    X(fdivs_S, K_F32, AR_S, "fdivs")                                                     \
    X(fdivl_S, K_F64, AR_S, "fdivl")                                                     \
    X(fdivrs_S, K_F32, AR_S, "fdivrs")                                                   \
    X(fdivrl_S, K_F64, AR_S, "fdivrl")                                                   \
    X(fadds_R, K_F32, AR_R, "fadds")                                                     \
    X(faddl_R, K_F64, AR_R, "faddl")                                                     \
    X(fsubl_R, K_F64, AR_R, "fsubl")                                                     \
    X(fsubrl_R, K_F64, AR_R, "fsubrl")                                                   \
    X(fmuls_R, K_F32, AR_R, "fmuls")                                                     \
    X(fmull_R, K_F64, AR_R, "fmull")                                                     \
    X(fdivl_R, K_F64, AR_R, "fdivl")                                                     \
    X(fdivrl_R, K_F64, AR_R, "fdivrl")                                                   \
    X(fiadds_S, K_I16, AR_S, "fiadds")                                                   \
    X(fiaddl_S, K_I32, AR_S, "fiaddl")                                                   \
    X(fisubs_S, K_I16, AR_S, "fisubs")                                                   \
    X(fisubl_S, K_I32, AR_S, "fisubl")                                                   \
    X(fisubrs_S, K_I16, AR_S, "fisubrs")                                                 \
    X(fisubrl_S, K_I32, AR_S, "fisubrl")                                                 \
    X(fimuls_S, K_I16, AR_S, "fimuls")                                                   \
    X(fimull_S, K_I32, AR_S, "fimull")                                                   \
    X(fidivs_S, K_I16, AR_S, "fidivs")                                                   \
    X(fidivl_S, K_I32, AR_S, "fidivl")                                                   \
    X(fidivrs_S, K_I16, AR_S, "fidivrs")                                                 \
    X(fidivrl_S, K_I32, AR_S, "fidivrl")                                                 \
    X(fiadds_R, K_I16, AR_R, "fiadds")                                                   \
    X(fiaddl_R, K_I32, AR_R, "fiaddl")                                                   \
    X(fisubl_R, K_I32, AR_R, "fisubl")                                                   \
    X(fisubrl_R, K_I32, AR_R, "fisubrl")                                                 \
    X(fimull_R, K_I32, AR_R, "fimull")                                                   \
    X(fidivl_R, K_I32, AR_R, "fidivl")                                                   \
    X(fidivrl_R, K_I32, AR_R, "fidivrl")                                                 \
    X(fiaddl_B, K_I32, AR_B, "fiaddl")                                                   \
    X(fmull_B, K_F64, AR_B, "fmull")                                                     \
    X(fcoms_S, K_F32, CM_S, "fcoms")                                                     \
    X(fcoml_S, K_F64, CM_S, "fcoml")                                                     \
    X(fcomps_S, K_F32, CM_S, "fcomps")                                                   \
    X(fcompl_S, K_F64, CM_S, "fcompl")                                                   \
    X(ficoms_S, K_I16, CM_S, "ficoms")                                                   \
    X(ficoml_S, K_I32, CM_S, "ficoml")                                                   \
    X(ficomps_S, K_I16, CM_S, "ficomps")                                                 \
    X(ficompl_S, K_I32, CM_S, "ficompl")                                                 \
    X(fcoms_R, K_F32, CM_R, "fcoms")                                                     \
    X(fcoml_R, K_F64, CM_R, "fcoml")                                                     \
    X(fcomps_R, K_F32, CM_R, "fcomps")                                                   \
    X(fcompl_R, K_F64, CM_R, "fcompl")                                                   \
    X(ficoms_R, K_I16, CM_R, "ficoms")                                                   \
    X(ficoml_R, K_I32, CM_R, "ficoml")                                                   \
    X(ficomps_R, K_I16, CM_R, "ficomps")                                                 \
    X(ficompl_R, K_I32, CM_R, "ficompl")                                                 \
    X(fsts_S, K_NONE, STO_S, "fsts")                                                     \
    X(fstl_S, K_NONE, STO_S, "fstl")                                                     \
    X(fstps_S, K_NONE, STO_S, "fstps")                                                   \
    X(fstpl_S, K_NONE, STO_S, "fstpl")                                                   \
    X(fstpt_S, K_NONE, STO_S, "fstpt")                                                   \
    X(fists_S, K_NONE, STO_S, "fists")                                                   \
    X(fistl_S, K_NONE, STO_S, "fistl")                                                   \
    X(fistps_S, K_NONE, STO_S, "fistps")                                                 \
    X(fistpl_S, K_NONE, STO_S, "fistpl")                                                 \
    X(fistpll_S, K_NONE, STO_S, "fistpll")                                               \
    X(fisttps_S, K_NONE, STO_S, "fisttps")                                               \
    X(fisttpl_S, K_NONE, STO_S, "fisttpl")                                               \
    X(fisttpll_S, K_NONE, STO_S, "fisttpll")                                             \
    X(fbstp_S, K_NONE, STO_S, "fbstp")                                                   \
    X(fsts_R, K_NONE, STO_R, "fsts")                                                     \
    X(fstl_R, K_NONE, STO_R, "fstl")                                                     \
    X(fists_R, K_NONE, STO_R, "fists")                                                   \
    X(fistl_R, K_NONE, STO_R, "fistl")                                                   \
    X(fstps_R, K_NONE, STO_R_P, "fstps")                                                 \
    X(fstpl_R, K_NONE, STO_R_P, "fstpl")                                                 \
    X(fstpt_R, K_NONE, STO_R_P, "fstpt")                                                 \
    X(fistps_R, K_NONE, STO_R_P, "fistps")                                               \
    X(fistpl_R, K_NONE, STO_R_P, "fistpl")                                               \
    X(fistpll_R, K_NONE, STO_R_P, "fistpll")                                             \
    X(fisttps_R, K_NONE, STO_R_P, "fisttps")                                             \
    X(fisttpl_R, K_NONE, STO_R_P, "fisttpl")                                             \
    X(fisttpll_R, K_NONE, STO_R_P, "fisttpll")                                           \
    X(fbstp_R, K_NONE, STO_R_P, "fbstp")                                                 \
    X(fstpl_B, K_NONE, STO_B_P, "fstpl")                                                 \
    X(fistpl_B, K_NONE, STO_B_P, "fistpl")                                               \
    X(fldcw_S, K_CW, CW_S, "")                                                           \
    X(fldcw_R, K_CW, CW_R, "")                                                           \
    X(fnstcw_S, K_NONE, STCW_S, "")                                                      \
    X(fnstcw_R, K_NONE, STCW_R, "")                                                      \
    X(fnstsw_S, K_NONE, STSW_S, "")                                                      \
    X(fnstsw_R, K_NONE, STSW_R, "")                                                      \
    X(fnsave_frstor_S, K_NONE, SAVE_S, "")                                               \
    X(fnsave_frstor_R, K_NONE, SAVE_R, "")                                               \
    X(fnsave_frstor_B, K_NONE, SAVE_B, "")                                               \
    X(fnstenv_fldenv_S, K_NONE, ENV_S, "")                                               \
    X(fistpl_pair_rc, K_NONE, RC_PAIR, "")                                               \
    X(fnstcw_fistpl_pair_rc, K_NONE, RC_PAIR_CW, "")                                     \
    X(fild_arith, K_I32, F_FILD_ARITH, "")                                               \
    X(fild_arith_fstp, K_I32, F_FILD_ARITH_FSTP, "")                                     \
    X(fld_fild_arith, K_I32, F_FLD_FILD_ARITH, "")                                       \
    X(fld_arithp_fldl, K_F64, F_FLD_ARITHP, "fldl")                                      \
    X(fld_arithp_fildl, K_I32, F_FLD_ARITHP, "fildl")                                    \
    X(fld_arith_arithp_fldl, K_F64, F_FLD_ARITH_ARITHP, "fldl")                          \
    X(fld_arith_arithp_filds, K_I16, F_FLD_ARITH_ARITHP, "filds")                        \
    X(fld_arithm_arithp, K_F64, F_FLD_ARITHM_ARITHP, "")                                 \
    X(fld_fstp_from_fldl, K_F64, F_FLD_FSTP_FROM, "fldl")                                \
    X(fld_fstp_from_fildl, K_I32, F_FLD_FSTP_FROM, "fildl")                              \
    X(fld_fstp_to_fstps, K_NONE, F_FLD_FSTP_TO, "fstps")                                 \
    X(fld_fstp_to_fstpl, K_NONE, F_FLD_FSTP_TO, "fstpl")                                 \
    X(fld_fcomp_fstsw_fldl, K_F64, F_FLD_FCOMP_FSTSW, "fldl")                            \
    X(fld_fcomp_fstsw_fildl, K_I32, F_FLD_FCOMP_FSTSW, "fildl")                          \
    X(fld_fcompm_fstsw, K_F64, F_FLD_FCOMPM_FSTSW, "fcompl")                             \
    X(fld_fcompm_fstsw_s, K_F32, F_FLD_FCOMPM_FSTSW, "fcomps")                           \
    X(fld_fcomp_fildl, K_I32, F_FLD_FCOMP, "fildl")                                      \
    X(fld_fcomp_fldl, K_F64, F_FLD_FCOMP, "fldl")                                        \
    X(fld_fcompp_fstsw_fldl, K_F64, F_FLD_FCOMPP_FSTSW, "fldl")                          \
    X(fld_fcompp_fstsw_fildl, K_I32, F_FLD_FCOMPP_FSTSW, "fildl")                        \
    X(fld_fld_fucompp_fldl, K_F64, F_FLD_FLD_FUCOMPP, "fldl")                            \
    X(fld_fld_fucompp_fildl, K_I32, F_FLD_FLD_FUCOMPP, "fildl")                          \
    X(arithp_fstp_l, K_NONE, F_ARITHP_FSTP, "fstpl")                                     \
    X(arithp_fstp_s, K_NONE, F_ARITHP_FSTP, "fstps")                                     \
    X(arith_fstp_faddl, K_F64, F_ARITH_FSTP, "faddl")                                    \
    X(arith_fstp_fmuls, K_F32, F_ARITH_FSTP, "fmuls")                                    \
    X(arith_fstpm_l, K_NONE, F_ARITH_FSTPM, "fstpl")                                     \
    X(fstp_arith_fstp, K_NONE, F_FSTP_ARITH_FSTP, "")                                    \
    X(arith_faddp_fmull, K_F64, F_ARITH_FADDP, "fmull")                                  \
    X(arith_faddp_fmuls, K_F32, F_ARITH_FADDP, "fmuls")                                  \
    X(fstp_fld_fldl, K_NONE, F_FSTP_FLD, "fldl")                                         \
    X(fstp_fld_fildl, K_NONE, F_FSTP_FLD, "fildl")                                       \
    X(fxch_fstp, K_NONE, F_FXCH_FSTP, "fstpl")                                           \
    X(fld_group, K_F64, F_FLD_GROUP, "")

// Returns 1 when %rcx, %rdx, %rsi and %rdi still hold their inputs.  All
// four are read-write operands, so a translator that writes one of them is
// seen here rather than corrupting the compiler's view of the registers.
typedef int (*CaseFn)(uintptr_t base, const double* in, unsigned char* out, uint16_t* sw);

#define DEFINE_VARIANT(name, suffix, shape, op, M, M4, M8, B)                            \
    static int name##_##suffix(uintptr_t base, const double* in, unsigned char* out,      \
                               uint16_t* sw) {                                           \
        uintptr_t rcx = base;                                                            \
        uintptr_t rdx = base + 8;                                                        \
        const double* rsi = in;                                                          \
        unsigned char* rdi = out;                                                        \
        uint16_t ax;                                                                     \
        __asm__ volatile("fninit\n\t" SEP shape(op, M, M4, M8, B) SEP "fnstsw %%ax\n\t"  \
                         "fninit\n\t"                                                    \
                         : "=a"(ax), "+c"(rcx), "+d"(rdx), "+S"(rsi), "+D"(rdi)          \
                         :                                                               \
                         : "r11", "cc", "memory");                                       \
        *sw = ax;                                                                        \
        return rcx == base && rdx == base + 8 && rsi == in && rdi == out;                \
    }

#define DEFINE_CASE(name, kind, shape, op)                                               \
    DEFINE_VARIANT(name, gs, shape, op, GS_M, GS_M4, GS_M8, GS_B)                        \
    DEFINE_VARIANT(name, fs, shape, op, FS_M, FS_M4, FS_M8, FS_B)                        \
    DEFINE_VARIANT(name, pl, shape, op, PL_M, PL_M4, PL_M8, PL_B)

CASES(DEFINE_CASE)

typedef struct {
    const char* name;
    enum InitKind kind;
    CaseFn gs;
    CaseFn fs;
    CaseFn pl;
} Case;

#define CASE_ENTRY(name, kind, shape, op) {#name, kind, name##_gs, name##_fs, name##_pl},

static const Case kCases[] = {CASES(CASE_ENTRY)};

// ── Fixed expected values ──────────────────────────────────────────────────
//
// Inputs are 1.75 and -3.5 at (%rsi).  fill() puts 3, -4, 5, ... (scaled by
// 0.625 for doubles) at the area.  Every value here is exact.

enum ExpectType { E_F64, E_I32, E_U16 };

typedef struct {
    const char* name;
    int in_area;  // 0: offset into out, 1: offset into the memory operand's area
    int off;
    enum ExpectType type;
    double d;
    int32_t i;
} Expect;

static const Expect kExpect[] = {
    {"fldl_S", 0, 0, E_F64, 1.875, 0},
    {"fildl_R", 0, 0, E_F64, 3.0, 0},
    {"fildl_R", 0, 16, E_F64, 1.75, 0},
    {"faddl_S", 0, 0, E_F64, 3.625, 0},
    {"fiaddl_R", 0, 0, E_F64, -0.5, 0},
    {"fiaddl_R", 0, 8, E_F64, 1.75, 0},
    {"fstpl_S", 1, 0, E_F64, 1.75, 0},
    {"fistpl_R", 1, 0, E_I32, 0.0, -4},
    {"fistpl_R", 0, 0, E_F64, 1.75, 0},
    {"fisttpl_S", 1, 0, E_I32, 0.0, 1},
    {"fbstp_S", 1, 0, E_I32, 0.0, 2},
    // -3.5 < 1.875: C0 set, TOP = 6 with two values loaded.
    {"fcoml_S", 0, 48, E_U16, 0.0, 0x3100},
    {"fistpl_pair_rc", 1, 0, E_I32, 0.0, -4},
    {"fistpl_pair_rc", 1, 4, E_I32, 0.0, 2},
    {"fild_arith", 0, 0, E_F64, -10.5, 0},
    {"fnsave_frstor_R", 0, 0, E_F64, -3.5, 0},
    {"fnsave_frstor_R", 0, 8, E_F64, 1.75, 0},
    // FLDCW to round toward zero, then FISTP -3.5.
    {"fldcw_R", 0, 32, E_I32, 0.0, -3},
};

// Returns the number of expected values for `name` that `out` and `area`
// miss, printing each.
static int check_expected(const char* name, const unsigned char* out, const unsigned char* area) {
    int bad = 0;
    for (size_t e = 0; e < sizeof(kExpect) / sizeof(kExpect[0]); e++) {
        const Expect* x = &kExpect[e];
        if (strcmp(x->name, name) != 0) {
            continue;
        }
        const unsigned char* p = (x->in_area ? area : out) + x->off;
        int ok;
        if (x->type == E_F64) {
            double got;
            memcpy(&got, p, 8);
            ok = memcmp(&got, &x->d, 8) == 0;
            if (!ok) {
                printf(" %s+%d=%g want %g", x->in_area ? "mem" : "out", x->off, got, x->d);
            }
        } else if (x->type == E_I32) {
            int32_t got;
            memcpy(&got, p, 4);
            ok = got == x->i;
            if (!ok) {
                printf(" %s+%d=%d want %d", x->in_area ? "mem" : "out", x->off, got, x->i);
            }
        } else {
            uint16_t got;
            memcpy(&got, p, 2);
            ok = got == (uint16_t)x->i;
            if (!ok) {
                printf(" %s+%d=%04x want %04x", x->in_area ? "mem" : "out", x->off, got,
                       (uint16_t)x->i);
            }
        }
        bad += !ok;
    }
    return bad;
}

// ── Area setup ─────────────────────────────────────────────────────────────

static pthread_key_t g_keys[kAreaQwords];
static unsigned char g_plain[kAreaBytes] __attribute__((aligned(16)));
static unsigned char g_fsarea[kAreaBytes] __attribute__((aligned(16)));

static void fill(unsigned char* area, enum InitKind kind) {
    memset(area, 0, kAreaBytes);
    for (int i = 0; i < kAreaBytes / 2; i++) {
        const int v = (i % 2) ? -(i + 3) : (i + 3);
        switch (kind) {
            case K_I16: {
                const int16_t x = (int16_t)v;
                memcpy(area + 2 * i, &x, 2);
                break;
            }
            case K_I32:
                if (i < kAreaBytes / 4) {
                    const int32_t x = v;
                    memcpy(area + 4 * i, &x, 4);
                }
                break;
            case K_I64:
                if (i < kAreaBytes / 8) {
                    const int64_t x = v;
                    memcpy(area + 8 * i, &x, 8);
                }
                break;
            case K_F32:
                if (i < kAreaBytes / 4) {
                    const float x = (float)v * 0.75f;
                    memcpy(area + 4 * i, &x, 4);
                }
                break;
            case K_F64:
                if (i < kAreaBytes / 8) {
                    const double x = (double)v * 0.625;
                    memcpy(area + 8 * i, &x, 8);
                }
                break;
            default:
                break;
        }
    }
    if (kind == K_F80) {
        // 2.5: mantissa 0xA000000000000000, exponent 0x4000.
        const uint64_t mant = 0xA000000000000000ULL;
        const uint16_t sexp = 0x4000;
        memcpy(area, &mant, 8);
        memcpy(area + 8, &sexp, 2);
    } else if (kind == K_BCD) {
        // -1234 packed BCD.
        area[0] = 0x34;
        area[1] = 0x12;
        area[9] = 0x80;
    } else if (kind == K_CW) {
        // Round toward zero, all exceptions masked.
        const uint16_t cw = 0x0C7F;
        memcpy(area, &cw, 2);
        memcpy(area + 8, &cw, 2);
    }
}

static void gs_write(const unsigned char* area) {
    for (int i = 0; i < kAreaQwords; i++) {
        uint64_t q;
        memcpy(&q, area + 8 * i, 8);
        pthread_setspecific(g_keys[i], (const void*)(uintptr_t)q);
    }
}

static void gs_read(unsigned char* area) {
    for (int i = 0; i < kAreaQwords; i++) {
        const uint64_t q = (uint64_t)(uintptr_t)pthread_getspecific(g_keys[i]);
        memcpy(area + 8 * i, &q, 8);
    }
}

static int setup_keys(void) {
    for (int i = 0; i < kAreaQwords; i++) {
        if (pthread_key_create(&g_keys[i], NULL) != 0) {
            return 0;
        }
        if (i > 0 && g_keys[i] != g_keys[0] + (pthread_key_t)i) {
            return 0;
        }
    }
    // The slots must be where %gs: addressing finds them.
    pthread_setspecific(g_keys[0], (const void*)(uintptr_t)0x1122334455667788ULL);
    uint64_t got;
    uintptr_t off = (uintptr_t)g_keys[0] * 8;
    __asm__ volatile("movq %%gs:(%1), %0" : "=r"(got) : "r"(off));
    return got == 0x1122334455667788ULL;
}

// The FS runs need a zero FS base.
static int fs_base_is_zero(void) {
    static volatile uint64_t probe = 0x8877665544332211ULL;
    uint64_t got;
    __asm__ volatile("movq %%fs:(%1), %0" : "=r"(got) : "r"(&probe) : "memory");
    return got == probe;
}

// Compares one segment run with the plain run; prints what differs.
static int same_as_plain(const char* seg, const unsigned char* out, const unsigned char* area,
                         uint16_t sw, const unsigned char* pl_out, uint16_t pl_sw) {
    const int ok_out = memcmp(out, pl_out, kOutBytes) == 0;
    const int ok_area = memcmp(area, g_plain, kAreaBytes) == 0;
    const int ok_sw = sw == pl_sw;
    if (ok_out && ok_area && ok_sw) {
        return 1;
    }
    printf(" [%s differs:%s%s%s", seg, ok_out ? "" : " out", ok_area ? "" : " mem",
           ok_sw ? "" : " sw");
    for (int b = 0; b < kOutBytes; b += 8) {
        if (memcmp(out + b, pl_out + b, 8) != 0) {
            uint64_t g;
            uint64_t p;
            memcpy(&g, out + b, 8);
            memcpy(&p, pl_out + b, 8);
            printf(" out+%d %s=%016llx plain=%016llx", b, seg, (unsigned long long)g,
                   (unsigned long long)p);
        }
    }
    if (!ok_sw) {
        printf(" sw %s=%04x plain=%04x", seg, sw, pl_sw);
    }
    printf("]");
    return 0;
}

// ---------------------------------------------------------------------------

// An optional argument runs only the case of that name.  Output is
// unbuffered and each name is printed before its case runs, so a
// translator abort names the case that caused it.
int main(int argc, char** argv) {
    setvbuf(stdout, NULL, _IONBF, 0);
    const char* only = (argc > 1) ? argv[1] : NULL;
    if (!setup_keys()) {
        printf("setup: consecutive TSD slots not reachable through %%gs  FAIL\n");
        return 1;
    }
    // A misspelt name in kExpect would silently check nothing.
    for (size_t e = 0; e < sizeof(kExpect) / sizeof(kExpect[0]); e++) {
        int found = 0;
        for (size_t c = 0; c < sizeof(kCases) / sizeof(kCases[0]); c++) {
            found |= strcmp(kExpect[e].name, kCases[c].name) == 0;
        }
        if (!found) {
            printf("setup: expected value for unknown case %s  FAIL\n", kExpect[e].name);
            return 1;
        }
    }
    const int fs_ok = fs_base_is_zero();
    if (!fs_ok) {
        printf("note: FS base is not zero, %%fs: runs skipped\n");
    }
    const uintptr_t gs_base = (uintptr_t)g_keys[0] * 8 - 8;
    const uintptr_t fs_base = (uintptr_t)g_fsarea - 8;
    const uintptr_t pl_base = (uintptr_t)g_plain - 8;
    const double in[4] = {1.75, -3.5, 0.0, 0.0};

    int pass = 0;
    int fail = 0;
    const int n = (int)(sizeof(kCases) / sizeof(kCases[0]));
    for (int c = 0; c < n; c++) {
        const Case* k = &kCases[c];
        if (only != NULL && strcmp(only, k->name) != 0) {
            continue;
        }
        printf("%-28s ", k->name);
        unsigned char init[kAreaBytes];
        unsigned char gs_area[kAreaBytes];
        unsigned char gs_out[kOutBytes];
        unsigned char fs_out[kOutBytes];
        unsigned char pl_out[kOutBytes];
        uint16_t gs_sw = 0;
        uint16_t fs_sw = 0;
        uint16_t pl_sw = 0;
        fill(init, k->kind);

        memcpy(g_plain, init, kAreaBytes);
        memset(pl_out, 0xEE, kOutBytes);
        const int pl_kept = k->pl(pl_base, in, pl_out, &pl_sw);

        gs_write(init);
        memset(gs_out, 0xEE, kOutBytes);
        const int gs_kept = k->gs(gs_base, in, gs_out, &gs_sw);
        gs_read(gs_area);

        int fs_kept = 1;
        if (fs_ok) {
            memcpy(g_fsarea, init, kAreaBytes);
            memset(fs_out, 0xEE, kOutBytes);
            fs_kept = k->fs(fs_base, in, fs_out, &fs_sw);
        }

        // Print the details first, then the verdict, so a failing line
        // still ends in FAIL for the harness.
        int ok = 1;
        if (!pl_kept || !gs_kept || !fs_kept) {
            printf(" [register clobbered:%s%s%s]", pl_kept ? "" : " plain", gs_kept ? "" : " gs",
                   fs_kept ? "" : " fs");
            ok = 0;
        }
        if (check_expected(k->name, pl_out, g_plain) != 0) {
            ok = 0;
        }
        if (!same_as_plain("gs", gs_out, gs_area, gs_sw, pl_out, pl_sw)) {
            ok = 0;
        }
        if (fs_ok && !same_as_plain("fs", fs_out, g_fsarea, fs_sw, pl_out, pl_sw)) {
            ok = 0;
        }
        printf("%s%s\n", ok ? "" : "  ", ok ? "PASS" : "FAIL");
        ok ? pass++ : fail++;
    }

    printf("\n%d/%d passed\n", pass, pass + fail);
    return (fail || pass == 0) ? 1 : 0;
}
