/*
 * test_arith_faddp.c — Tests for FMUL + FADDP/FSUBP/FSUBRP → FMA fusion.
 *
 * Covers: fmul(reg)+faddp, fmul(reg)+fsubp, fmul(reg)+fsubrp,
 *         fmul(mem32)+faddp, fmul(mem64)+faddp,
 *         stack depth preservation, consecutive pairs,
 *         mul_src == accumulator (ST(1)), squaring (ST(0)*ST(0)).
 *
 * Build: clang -arch x86_64 -O0 -g -o test_arith_faddp test_arith_faddp.c
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

static void check(const char* name, double got, double expected) {
    if (as_u64(got) != as_u64(expected)) {
        printf("FAIL  %-55s  got=%.17g  expected=%.17g\n", name, got, expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

/* ==== FMUL ST(0),ST(i) + FADDP: multiply-accumulate ==== */
static double fmul_reg_faddp(void) {
    double r;
    /* Stack: push 2.0, push 5.0, push 3.0
     * ST(0)=3.0, ST(1)=5.0, ST(2)=2.0
     * FMUL ST(0),ST(1): ST(0) = 3.0 * 5.0 = 15.0
     * FADDP:            ST(1) = ST(1) + ST(0) = 5.0 + 15.0 = 20.0, pop
     *   Wait — FADDP operates on the NEW ST(0) (=15.0) and ST(1) (=5.0).
     *   Actually after FMUL, ST(0)=15.0, ST(1)=5.0, ST(2)=2.0
     *   But FADDP default is FADDP ST(1),ST(0): ST(1)=ST(1)+ST(0)=5.0+15.0=20.0, pop
     *   After pop: ST(0)=20.0, ST(1)=2.0
     *   Store 20.0, then clean up.
     *
     * Hmm, for the fusion to fire, we need the pattern FMUL (not preceded by
     * FLD that would form fld_arith_arithp). Let's set up values already on
     * the stack and have FMUL operate on registers.
     */
    __asm__ volatile(
        "fldl %1\n\t"            /* ST(0) = 2.0 (accumulator, will be ST(2)) */
        "fldl %2\n\t"            /* ST(0) = 5.0, ST(1) = 2.0 */
        "fldl %3\n\t"            /* ST(0) = 3.0, ST(1) = 5.0, ST(2) = 2.0 */
        "fmul %%st(1), %%st\n\t" /* ST(0) = 3*5 = 15, ST(1) = 5, ST(2) = 2 */
        "faddp\n\t"              /* ST(0) = 5+15 = 20, ST(1) = 2 */
        "fstpl %0\n\t"           /* r = 20, pop -> ST(0) = 2 */
        "fstp %%st(0)\n"         /* clean stack */
        : "=m"(r)
        : "m"((double){2.0}), "m"((double){5.0}), "m"((double){3.0}));
    return r;
}

/* ==== FMUL ST(0),ST(i) + FSUBP ==== */
static double fmul_reg_fsubp(void) {
    double r;
    /* ST(0)=3, ST(1)=5, ST(2)=100
     * FMUL ST(0),ST(1): ST(0) = 3*5 = 15
     * GAS fsubp → Intel FSUBRP: ST(1) = ST(0) - ST(1) = 15 - 5 = 10, pop
     * After pop: ST(0)=10, ST(1)=100
     */
    __asm__ volatile(
        "fldl %1\n\t"            /* ST(0) = 100 */
        "fldl %2\n\t"            /* ST(0) = 5, ST(1) = 100 */
        "fldl %3\n\t"            /* ST(0) = 3, ST(1) = 5, ST(2) = 100 */
        "fmul %%st(1), %%st\n\t" /* ST(0) = 15, ST(1) = 5, ST(2) = 100 */
        "fsubp\n\t"              /* GAS fsubp = Intel fsubrp: ST(1)=ST(0)-ST(1)=15-5=10, pop */
        "fstpl %0\n\t"
        "fstp %%st(0)\n"
        : "=m"(r)
        : "m"((double){100.0}), "m"((double){5.0}), "m"((double){3.0}));
    return r;
}

/* ==== FMUL ST(0),ST(i) + FSUBRP ==== */
static double fmul_reg_fsubrp(void) {
    double r;
    /* ST(0)=3, ST(1)=5, ST(2)=100
     * FMUL ST(0),ST(1): ST(0) = 15
     * GAS fsubrp → Intel FSUBP: ST(1) = ST(1) - ST(0) = 5 - 15 = -10, pop
     * After pop: ST(0)=-10, ST(1)=100
     */
    __asm__ volatile(
        "fldl %1\n\t"            /* ST(0) = 100 */
        "fldl %2\n\t"            /* ST(0) = 5, ST(1) = 100 */
        "fldl %3\n\t"            /* ST(0) = 3, ST(1) = 5, ST(2) = 100 */
        "fmul %%st(1), %%st\n\t" /* ST(0) = 15 */
        "fsubrp\n\t"             /* GAS fsubrp = Intel fsubp: ST(1)=ST(1)-ST(0)=5-15=-10, pop */
        "fstpl %0\n\t"
        "fstp %%st(0)\n"
        : "=m"(r)
        : "m"((double){100.0}), "m"((double){5.0}), "m"((double){3.0}));
    return r;
}

/* ==== FMUL mem32 + FADDP: memory f32 source ==== */
static double fmul_mem32_faddp(void) {
    double r;
    float f32_val = 4.0f;
    /* ST(0)=10, ST(1)=7
     * FMULS [f32_val]: ST(0) = 10*4 = 40
     * FADDP: ST(1) = 7+40 = 47, pop
     */
    __asm__ volatile(
        "fldl %1\n\t"  /* ST(0) = 7 */
        "fldl %2\n\t"  /* ST(0) = 10, ST(1) = 7 */
        "fmuls %3\n\t" /* ST(0) = 10*4 = 40, ST(1) = 7 */
        "faddp\n\t"    /* ST(0) = 7+40 = 47 */
        "fstpl %0\n"
        : "=m"(r)
        : "m"((double){7.0}), "m"((double){10.0}), "m"(f32_val));
    return r;
}

/* ==== FMUL mem64 + FADDP: memory f64 source ==== */
static double fmul_mem64_faddp(void) {
    double r;
    double f64_val = 3.0;
    /* ST(0)=5, ST(1)=2
     * FMULL [f64_val]: ST(0) = 5*3 = 15
     * FADDP: ST(1) = 2+15 = 17, pop
     */
    __asm__ volatile(
        "fldl %1\n\t"  /* ST(0) = 2 */
        "fldl %2\n\t"  /* ST(0) = 5, ST(1) = 2 */
        "fmull %3\n\t" /* ST(0) = 15, ST(1) = 2 */
        "faddp\n\t"    /* ST(0) = 17 */
        "fstpl %0\n"
        : "=m"(r)
        : "m"((double){2.0}), "m"((double){5.0}), "m"(f64_val));
    return r;
}

/* ==== Stack depth preserved: deeper values survive ==== */
static void stack_depth(double* out_result, double* out_remaining) {
    /* ST(0)=3, ST(1)=5, ST(2)=100
     * FMUL ST(0),ST(1): ST(0) = 15
     * FADDP: ST(0) = 5+15 = 20, ST(1) = 100
     */
    __asm__ volatile(
        "fldl %2\n\t"            /* ST(0) = 100 */
        "fldl %3\n\t"            /* ST(0) = 5, ST(1) = 100 */
        "fldl %4\n\t"            /* ST(0) = 3, ST(1) = 5, ST(2) = 100 */
        "fmul %%st(1), %%st\n\t" /* ST(0) = 15, ST(1) = 5, ST(2) = 100 */
        "faddp\n\t"              /* ST(0) = 20, ST(1) = 100 */
        "fstpl %0\n\t"           /* result = 20, pop -> ST(0) = 100 */
        "fstpl %1\n"             /* remaining = 100 */
        : "=m"(*out_result), "=m"(*out_remaining)
        : "m"((double){100.0}), "m"((double){5.0}), "m"((double){3.0}));
}

/* ==== Consecutive FMUL+FADDP pairs ==== */
static void consecutive_pairs(double* out0, double* out1) {
    __asm__ volatile(
        /* First pair: 3*5 + 2 = 17 */
        "fldl %2\n\t"            /* ST(0) = 2 */
        "fldl %3\n\t"            /* ST(0) = 5, ST(1) = 2 */
        "fldl %4\n\t"            /* ST(0) = 3, ST(1) = 5, ST(2) = 2 */
        "fmul %%st(1), %%st\n\t" /* ST(0) = 15, ST(1) = 5, ST(2) = 2 */
        "faddp\n\t"              /* ST(0) = 20, ST(1) = 2 */
        "fstpl %0\n\t"           /* out0 = 20, ST(0) = 2 */
        "fstp %%st(0)\n\t"       /* clean */
        /* Second pair: 4*2 + 10 = 18 */
        "fldl %5\n\t"            /* ST(0) = 10 */
        "fldl %6\n\t"            /* ST(0) = 2, ST(1) = 10 */
        "fldl %7\n\t"            /* ST(0) = 4, ST(1) = 2, ST(2) = 10 */
        "fmul %%st(1), %%st\n\t" /* ST(0) = 8, ST(1) = 2, ST(2) = 10 */
        "faddp\n\t"              /* ST(0) = 10, ST(1) = 10 */
        "fstpl %1\n\t"           /* out1 = 10 */
        "fstp %%st(0)\n"
        : "=m"(*out0), "=m"(*out1)
        : "m"((double){2.0}), "m"((double){5.0}), "m"((double){3.0}), "m"((double){10.0}),
          "m"((double){2.0}), "m"((double){4.0}));
}

/* ==== FMUL ST(0),ST(1) + FADDP: mul source == accumulator (both ST(1)) ==== */
static double mul_src_eq_accum(void) {
    double r;
    /* ST(0)=3, ST(1)=5
     * FMUL ST(0),ST(1): ST(0) = 3*5 = 15
     * FADDP: ST(0) = 5+15 = 20
     */
    __asm__ volatile(
        "fldl %1\n\t"            /* ST(0) = 5 */
        "fldl %2\n\t"            /* ST(0) = 3, ST(1) = 5 */
        "fmul %%st(1), %%st\n\t" /* ST(0) = 15, ST(1) = 5 */
        "faddp\n\t"              /* ST(0) = 20 */
        "fstpl %0\n"
        : "=m"(r)
        : "m"((double){5.0}), "m"((double){3.0}));
    return r;
}

/* ==== FMUL ST(0),ST(0) + FADDP: squaring ==== */
static double fmul_st0_st0_faddp(void) {
    double r;
    /* ST(0)=4, ST(1)=10
     * FMUL ST(0),ST(0): ST(0) = 4*4 = 16
     * FADDP: ST(0) = 10+16 = 26
     */
    __asm__ volatile(
        "fldl %1\n\t"            /* ST(0) = 10 */
        "fldl %2\n\t"            /* ST(0) = 4, ST(1) = 10 */
        "fmul %%st(0), %%st\n\t" /* ST(0) = 16, ST(1) = 10 */
        "faddp\n\t"              /* ST(0) = 26 */
        "fstpl %0\n"
        : "=m"(r)
        : "m"((double){10.0}), "m"((double){4.0}));
    return r;
}

/* ==== FMUL mem64 + FSUBP (GAS fsubp = Intel fsubrp) ==== */
static double fmul_mem64_fsubp(void) {
    double r;
    double f64_val = 3.0;
    /* ST(0)=5, ST(1)=2
     * FMULL [3.0]: ST(0) = 15
     * GAS fsubp = Intel fsubrp: ST(1) = ST(0) - ST(1) = 15 - 2 = 13, pop
     */
    __asm__ volatile(
        "fldl %1\n\t"
        "fldl %2\n\t"
        "fmull %3\n\t"
        "fsubp\n\t"
        "fstpl %0\n"
        : "=m"(r)
        : "m"((double){2.0}), "m"((double){5.0}), "m"(f64_val));
    return r;
}

/* ==== FMUL with deeper register + FADDP ==== */
static double fmul_deep_reg_faddp(void) {
    double r;
    /* ST(0)=3, ST(1)=5, ST(2)=7
     * FMUL ST(0),ST(2): ST(0) = 3*7 = 21
     * FADDP: ST(0) = 5+21 = 26, ST(1) = 7
     */
    __asm__ volatile(
        "fldl %1\n\t"            /* ST(0) = 7 */
        "fldl %2\n\t"            /* ST(0) = 5, ST(1) = 7 */
        "fldl %3\n\t"            /* ST(0) = 3, ST(1) = 5, ST(2) = 7 */
        "fmul %%st(2), %%st\n\t" /* ST(0) = 21 */
        "faddp\n\t"              /* ST(0) = 26, ST(1) = 7 */
        "fstpl %0\n\t"
        "fstp %%st(0)\n"
        : "=m"(r)
        : "m"((double){7.0}), "m"((double){5.0}), "m"((double){3.0}));
    return r;
}

int main(void) {
    /* FMUL reg + FADDP */
    check("fmul_reg+faddp: 3*5+5=20", fmul_reg_faddp(), 20.0);

    /* FMUL reg + FSUBP (GAS fsubp = Intel fsubrp: ST(0)-ST(1)) */
    check("fmul_reg+fsubp: 3*5-5=10", fmul_reg_fsubp(), 10.0);

    /* FMUL reg + FSUBRP (GAS fsubrp = Intel fsubp: ST(1)-ST(0)) */
    check("fmul_reg+fsubrp: 5-3*5=-10", fmul_reg_fsubrp(), -10.0);

    /* FMUL mem32 + FADDP */
    check("fmul_mem32+faddp: 10*4+7=47", fmul_mem32_faddp(), 47.0);

    /* FMUL mem64 + FADDP */
    check("fmul_mem64+faddp: 5*3+2=17", fmul_mem64_faddp(), 17.0);

    /* Stack depth preserved */
    {
        double result, remaining;
        stack_depth(&result, &remaining);
        check("depth: result (3*5+5=20)", result, 20.0);
        check("depth: remaining ST (100)", remaining, 100.0);
    }

    /* Consecutive pairs */
    {
        double a, b;
        consecutive_pairs(&a, &b);
        check("consecutive: first (3*5+5=20)", a, 20.0);
        check("consecutive: second (4*2+2=10)", b, 10.0);
    }

    /* mul source == accumulator */
    check("mul_src_eq_accum: 3*5+5=20", mul_src_eq_accum(), 20.0);

    /* squaring: ST(0)*ST(0) + ST(1) */
    check("fmul_st0_st0+faddp: 4*4+10=26", fmul_st0_st0_faddp(), 26.0);

    /* FMUL mem64 + FSUBP */
    check("fmul_mem64+fsubp: 5*3-2=13", fmul_mem64_fsubp(), 13.0);

    /* FMUL deep register + FADDP */
    check("fmul_deep_reg+faddp: 3*7+5=26", fmul_deep_reg_faddp(), 26.0);

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
