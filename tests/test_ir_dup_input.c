/*
 * test_ir_dup_input.c: two-operand transcendentals whose operands are the
 * same value.
 *
 * `fld %st(0)` and `fst %st(i)` copy a stack slot, so the IR gives both
 * operands of the following fyl2x, fpatan or fscale the same node.  The
 * lowering frees that node's FPR once before the inline polynomial runs, so
 * the IR pressure model has to count it once too.  Counting it twice
 * undercounts the polynomial's peak by one FPR: on an eight-register pool
 * the gate then lets the run through with exactly the pool size predicted
 * and the lowering runs out of FPRs.
 *
 * Each case keeps the fewest other values live that reach the pool size
 * under the double count: one for fyl2x, three for fpatan, five for fscale.
 * The live pool depends on stock's FPR seeding, so
 * tests/data/ir_dup_input.ir holds these blocks for ir_pressure_replay,
 * which run_tests.sh replays on an 8-FPR pool.
 *
 * Build: clang -arch x86_64 -O0 -o test_ir_dup_input test_ir_dup_input.c -lm
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MAX_ULP 4

static int failures = 0;

static void check_ulp(const char* name, double got, double expected) {
    uint64_t g, e;
    memcpy(&g, &got, sizeof(g));
    memcpy(&e, &expected, sizeof(e));
    if (g == e) {
        printf("PASS  %s\n", name);
        return;
    }
    uint64_t g_abs = g & 0x7fffffffffffffffULL;
    uint64_t e_abs = e & 0x7fffffffffffffffULL;
    uint64_t ulp_delta;
    if ((g >> 63) == (e >> 63)) {
        ulp_delta = (g_abs > e_abs) ? (g_abs - e_abs) : (e_abs - g_abs);
    } else {
        ulp_delta = g_abs + e_abs;
    }
    if (ulp_delta <= MAX_ULP) {
        printf("PASS  %s  [ulp=%llu]\n", name, (unsigned long long)ulp_delta);
        return;
    }
    printf("FAIL  %-45s  got=%.17g  expected=%.17g  ulp=%llu\n", name, got, expected,
           (unsigned long long)ulp_delta);
    failures++;
}

/* a + x*log2(x), with fld %st(0) feeding both fyl2x operands. */
static double fyl2x_fld_st0(const double* v) {
    double out;
    __asm__ volatile(
        "fldl   (%1)\n\t"  /* a */
        "fldl   8(%1)\n\t" /* x */
        "fld    %%st(0)\n\t"
        "fyl2x\n\t"
        "faddp\n\t"
        "fstpl  %0\n\t"
        : "=m"(out)
        : "r"(v)
        : "st", "st(1)", "st(2)");
    return out;
}

/* Same with fst %st(1) making the copy: b is overwritten before its use. */
static double fyl2x_fst_st1(const double* v) {
    double out;
    __asm__ volatile(
        "fldl   (%1)\n\t"   /* a */
        "fldl   16(%1)\n\t" /* b, replaced by x below */
        "fldl   8(%1)\n\t"  /* x */
        "fst    %%st(1)\n\t"
        "fyl2x\n\t"
        "faddp\n\t"
        "fstpl  %0\n\t"
        : "=m"(out)
        : "r"(v)
        : "st", "st(1)", "st(2)");
    return out;
}

/* a + b + c + atan2(x, x), with three values live across the fpatan. */
static double fpatan_fld_st0(const double* v) {
    double out;
    __asm__ volatile(
        "fldl   (%1)\n\t"   /* a */
        "fldl   8(%1)\n\t"  /* b */
        "fldl   16(%1)\n\t" /* c */
        "fldl   24(%1)\n\t" /* x */
        "fld    %%st(0)\n\t"
        "fpatan\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "fstpl  %0\n\t"
        : "=m"(out)
        : "r"(v)
        : "st", "st(1)", "st(2)", "st(3)", "st(4)");
    return out;
}

/* a + b + c + d + e + x*2^trunc(x), with five values live across fscale. */
static double fscale_fld_st0(const double* v) {
    double out;
    __asm__ volatile(
        "fldl   (%1)\n\t"   /* a */
        "fldl   8(%1)\n\t"  /* b */
        "fldl   16(%1)\n\t" /* c */
        "fldl   24(%1)\n\t" /* d */
        "fldl   32(%1)\n\t" /* e */
        "fldl   40(%1)\n\t" /* x */
        "fld    %%st(0)\n\t"
        "fscale\n\t"
        "fstp   %%st(1)\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "fstpl  %0\n\t"
        : "=m"(out)
        : "r"(v)
        : "st", "st(1)", "st(2)", "st(3)", "st(4)", "st(5)", "st(6)");
    return out;
}

int main(void) {
    {
        const double v[3] = {0.25, 8.0, -5.0};
        check_ulp("fyl2x  fld st(0)  0.25 + 8*log2(8)", fyl2x_fld_st0(v), 0.25 + 24.0);
        check_ulp("fyl2x  fst st(1)  0.25 + 8*log2(8)", fyl2x_fst_st1(v), 0.25 + 24.0);
    }
    {
        const double v[3] = {0.25, 3.0, -5.0};
        const double expected = 0.25 + 3.0 * log2(3.0);
        check_ulp("fyl2x  fld st(0)  0.25 + 3*log2(3)", fyl2x_fld_st0(v), expected);
    }
    {
        const double v[4] = {1.0, 2.0, 4.0, 1.0};
        const double expected = 1.0 + (2.0 + (4.0 + atan2(1.0, 1.0)));
        check_ulp("fpatan fld st(0)  7 + atan2(1, 1)", fpatan_fld_st0(v), expected);
    }
    {
        const double v[4] = {1.0, 2.0, 4.0, -2.5};
        const double expected = 1.0 + (2.0 + (4.0 + atan2(-2.5, -2.5)));
        check_ulp("fpatan fld st(0)  7 + atan2(-2.5, -2.5)", fpatan_fld_st0(v), expected);
    }
    {
        /* The values below the operand stay on the stack across fscale; the
         * fstp %st(1) drops the copy fscale leaves in ST(1). */
        const double v[6] = {1.0, 2.0, 4.0, 8.0, 16.0, 3.0};
        const double expected = 1.0 + (2.0 + (4.0 + (8.0 + (16.0 + 24.0))));
        check_ulp("fscale fld st(0)  31 + 3*2^3", fscale_fld_st0(v), expected);
    }
    {
        const double v[6] = {1.0, 2.0, 4.0, 8.0, 16.0, -2.75};
        const double expected = 1.0 + (2.0 + (4.0 + (8.0 + (16.0 + -2.75 / 4.0))));
        check_ulp("fscale fld st(0)  31 + -2.75*2^-2", fscale_fld_st0(v), expected);
    }

    printf("%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
