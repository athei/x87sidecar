/*
 * test_fma_pc53.c -- fmul followed by fadd/fsub must round the product
 * before the add, the way real x87 does.
 *
 * Windows processes run with the x87 control word at 0x027F, precision
 * control 53 bits.  At that setting a double multiply followed by a double
 * add is bit-exact against real x87 (and against stock Rosetta's f80
 * path), while a fused multiply-add is not: FMA keeps the infinitely
 * precise product, so a result that cancels to zero under x87 comes out
 * as the product's rounding error under FMA.
 *
 * Every case below sets PC=53 and picks operands so that the two forms
 * differ.  The expected values are what real x87 produces; run_tests.sh
 * phase 1 validates them under stock Rosetta.  The sequences are chosen to
 * cover each place the JIT could contract: the IR pass (runs of 3+ ops),
 * the fld_arith_arithp fusion (FLD, FMUL ST(0),ST(1), FADDP/FSUBP) and
 * the arith_faddp peephole (FMUL m64, FADDP), which the IR-off and
 * fusions-off phases reach in turn.
 *
 * Build: clang -arch x86_64 -O0 -g -o test_fma_pc53 test_fma_pc53.c
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static uint64_t as_u64(double d) {
    uint64_t u;
    memcpy(&u, &d, 8);
    return u;
}

static void check_f64(const char* name, double got, double expected) {
    if (as_u64(got) != as_u64(expected)) {
        printf("FAIL  %-58s  got=%.17g (0x%016llx)  expected=%.17g (0x%016llx)\n", name, got,
               (unsigned long long)as_u64(got), expected, (unsigned long long)as_u64(expected));
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

/* Windows default control word: all exceptions masked, PC=53, RC=nearest. */
#define CW_WINDOWS 0x027F

static void set_cw(uint16_t cw) {
    __asm__ volatile("fldcw %0" : : "m"(cw));
}

/* a*b = 1 - 2^-60 exactly.  Rounded to 53 bits it is 1.0, so a*b - 1
 * cancels to +0.0.  A fused multiply-subtract yields -2^-60 instead. */
static const double kA = 1.0 + 0x1p-30;
static const double kB = 1.0 - 0x1p-30;
static const double kOne = 1.0;
static const double kMinusOne = -1.0;

/* ── IR path: runs of 3+ x87 ops, memory operands ─────────────────────── */

/* FLD a; FMUL [b]; FSUB [1]; FSTP  ->  a*b - 1 */
static double ir_mul_sub(void) {
    double r;
    __asm__ volatile(
        "fldl %1\n"
        "fmull %2\n"
        "fsubl %3\n"
        "fstpl %0\n"
        : "=m"(r)
        : "m"(kA), "m"(kB), "m"(kOne));
    return r;
}

/* FLD a; FMUL [b]; FSUBR [1]; FSTP  ->  1 - a*b */
static double ir_mul_subr(void) {
    double r;
    __asm__ volatile(
        "fldl %1\n"
        "fmull %2\n"
        "fsubrl %3\n"
        "fstpl %0\n"
        : "=m"(r)
        : "m"(kA), "m"(kB), "m"(kOne));
    return r;
}

/* FLD a; FMUL [b]; FADD [-1]; FSTP  ->  a*b + (-1) */
static double ir_mul_add(void) {
    double r;
    __asm__ volatile(
        "fldl %1\n"
        "fmull %2\n"
        "faddl %3\n"
        "fstpl %0\n"
        : "=m"(r)
        : "m"(kA), "m"(kB), "m"(kMinusOne));
    return r;
}

/* ── arith_faddp peephole: FMUL m64 + FADDP ───────────────────────────── */

/* FLD -1; FLD a; FMUL [b]; FADDP; FSTP  ->  -1 + a*b */
static double peep_fmul_faddp(void) {
    double r;
    __asm__ volatile(
        "fldl %3\n"
        "fldl %1\n"
        "fmull %2\n"
        "faddp %%st(0), %%st(1)\n"
        "fstpl %0\n"
        : "=m"(r)
        : "m"(kA), "m"(kB), "m"(kMinusOne));
    return r;
}

/* FLD 1; FLD a; FMUL [b]; FSUBRP; FSTP  ->  a*b - 1
 * (GAS `fsubp` is Intel FSUBRP: ST(1) = ST(0) - ST(1).) */
static double peep_fmul_fsubrp(void) {
    double r;
    __asm__ volatile(
        "fldl %3\n"
        "fldl %1\n"
        "fmull %2\n"
        "fsubp %%st(0), %%st(1)\n"
        "fstpl %0\n"
        : "=m"(r)
        : "m"(kA), "m"(kB), "m"(kOne));
    return r;
}

/* ── fld_arith_arithp fusion: FLD, FMUL ST(0),ST(1), FADDP/FSUBP ──────── */

/* The multiplier is the value already on the stack, so the operands are
 * x = 1 + 2^-52 and a = 1 - 2^-52: a*x = 1 - 2^-104, which rounds to 1.0
 * at 53 bits but stays exact under FMA.  x + (-a)*x = 2^-52 under x87 and
 * 2^-52 + 2^-104 fused; a*x - x = -2^-52 under x87 and -2^-52 - 2^-104
 * fused.  Both fused values are representable, so the test distinguishes. */
static const double kX = 1.0 + 0x1p-52;
static const double kNegA = -(1.0 - 0x1p-52);
static const double kPosA = 1.0 - 0x1p-52;

/* FLD x; FLD -a; FMUL ST(0),ST(1); FADDP; FSTP  ->  x + (-a)*x */
static double fusion_fld_fmul_faddp(void) {
    double r;
    __asm__ volatile(
        "fldl %1\n"
        "fldl %2\n"
        "fmul %%st(1), %%st(0)\n"
        "faddp %%st(0), %%st(1)\n"
        "fstpl %0\n"
        : "=m"(r)
        : "m"(kX), "m"(kNegA));
    return r;
}

/* FLD x; FLD a; FMUL ST(0),ST(1); FSUBRP; FSTP  ->  a*x - x */
static double fusion_fld_fmul_fsubrp(void) {
    double r;
    __asm__ volatile(
        "fldl %1\n"
        "fldl %2\n"
        "fmul %%st(1), %%st(0)\n"
        "fsubp %%st(0), %%st(1)\n"
        "fstpl %0\n"
        : "=m"(r)
        : "m"(kX), "m"(kPosA));
    return r;
}

int main(void) {
    set_cw(CW_WINDOWS);

    printf("=== IR path (fmul + fadd/fsub, memory operands) ===\n");
    check_f64("A1 a*b - 1 == +0", ir_mul_sub(), 0.0);
    check_f64("A2 1 - a*b == +0", ir_mul_subr(), 0.0);
    check_f64("A3 a*b + (-1) == +0", ir_mul_add(), 0.0);

    printf("\n=== arith_faddp peephole (FMUL m64 + FADDP/FSUBRP) ===\n");
    check_f64("B1 -1 + a*b == +0", peep_fmul_faddp(), 0.0);
    check_f64("B2 a*b - 1 == +0", peep_fmul_fsubrp(), 0.0);

    printf("\n=== fld_arith_arithp fusion (FLD + FMUL ST(0),ST(1) + FADDP/FSUBRP) ===\n");
    check_f64("C1 x + (-a)*x == 2^-52", fusion_fld_fmul_faddp(), 0x1p-52);
    check_f64("C2 a*x - x == -2^-52", fusion_fld_fmul_fsubrp(), -0x1p-52);

    printf("\n%s  (%d failure%s)\n", failures == 0 ? "ALL PASS" : "SOME FAILURES", failures,
           failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
