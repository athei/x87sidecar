/*
 * test_fld_arith_arithp_fma.c -- Tests for the FMA fast path inside the
 * fld_arith_arithp fusion (FLD + FMUL + FADDP/FSUBP/FSUBRP → single FMA).
 *
 * The 3-instruction pattern FLD src / FMUL ST(0),ST(1) / ARITHp ST(1) is
 * handled by try_fuse_fld_arith_arithp.  When the middle op is FMUL and the
 * final op is FADDP/FSUBP/FSUBRP, the JIT can emit a single ARM64 FMA
 * instead of two separate FP instructions.
 *
 * Build: clang -arch x86_64 -O0 -g -o test_fld_arith_arithp_fma \
 *        test_fld_arith_arithp_fma.c
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
static uint32_t as_u32(float f) {
    uint32_t u;
    memcpy(&u, &f, 4);
    return u;
}

static void check_f64(const char* name, double got, double expected) {
    if (as_u64(got) != as_u64(expected)) {
        printf("FAIL  %-60s  got=%.17g  expected=%.17g\n", name, got, expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

static void check_f32(const char* name, float got, float expected) {
    if (as_u32(got) != as_u32(expected)) {
        printf("FAIL  %-60s  got=%.9g (0x%08x)  expected=%.9g (0x%08x)\n", name, got, as_u32(got),
               expected, as_u32(expected));
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

/* =========================================================================
 * Section A: FLD m64 + FMUL reg + FADDP/FSUBP/FSUBRP (register FMA paths)
 * ========================================================================= */

/* FLD m64 + FMUL ST(0),ST(1) + FADDP
 * ST(0)=4.0.  FLD [3.0] -> ST(0)=3, ST(1)=4.
 * FMUL ST(0),ST(1): ST(0) = 3*4 = 12.
 * FADDP ST(1): ST(1) = 4+12 = 16, pop.
 * Result: 16.0.
 */
static double test_a_fld_m64_fmul_faddp(void) {
    double src = 3.0;
    double result;
    __asm__ volatile(
        "fld1\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n" /* 4.0 */
        "fldl %1\n"
        "fmul %%st(1), %%st(0)\n"
        "faddp %%st(0), %%st(1)\n"
        "fstpl %0\n"
        : "=m"(result)
        : "m"(src));
    return result;
}

/* FLD m64 + FMUL ST(0),ST(1) + FSUBP
 * GAS fsubp = Intel FSUBRP: ST(1) = ST(0) - ST(1).
 * ST(0)=4.0.  FLD [3.0] -> ST(0)=3, ST(1)=4.
 * FMUL: ST(0) = 3*4 = 12.
 * FSUBRP: ST(1) = ST(0) - ST(1) = 12 - 4 = 8, pop.
 * Result: 8.0.
 */
static double test_a_fld_m64_fmul_fsubp(void) {
    double src = 3.0;
    double result;
    __asm__ volatile(
        "fld1\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n" /* 4.0 */
        "fldl %1\n"
        "fmul %%st(1), %%st(0)\n"
        "fsubp %%st(0), %%st(1)\n" /* GAS fsubp = Intel FSUBRP */
        "fstpl %0\n"
        : "=m"(result)
        : "m"(src));
    return result;
}

/* FLD m64 + FMUL ST(0),ST(1) + FSUBRP
 * GAS fsubrp = Intel FSUBP: ST(1) = ST(1) - ST(0).
 * ST(0)=4.0.  FLD [3.0] -> ST(0)=3, ST(1)=4.
 * FMUL: ST(0) = 3*4 = 12.
 * FSUBP: ST(1) = ST(1) - ST(0) = 4 - 12 = -8, pop.
 * Result: -8.0.
 */
static double test_a_fld_m64_fmul_fsubrp(void) {
    double src = 3.0;
    double result;
    __asm__ volatile(
        "fld1\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n" /* 4.0 */
        "fldl %1\n"
        "fmul %%st(1), %%st(0)\n"
        "fsubrp %%st(0), %%st(1)\n" /* GAS fsubrp = Intel FSUBP */
        "fstpl %0\n"
        : "=m"(result)
        : "m"(src));
    return result;
}

/* =========================================================================
 * Section B: FLD source variants (f32, register, constant, integer)
 * ========================================================================= */

/* FLD m32 + FMUL + FADDP
 * ST(0)=5.0.  FLD [2.0f] -> ST(0)=2, ST(1)=5.
 * FMUL: ST(0) = 2*5 = 10.
 * FADDP: ST(1) = 5+10 = 15, pop.
 * Result: 15.0.
 */
static double test_b_fld_m32_fmul_faddp(void) {
    float src = 2.0f;
    double result;
    __asm__ volatile(
        "fld1\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n" /* 5.0 */
        "flds %1\n"
        "fmul %%st(1), %%st(0)\n"
        "faddp %%st(0), %%st(1)\n"
        "fstpl %0\n"
        : "=m"(result)
        : "m"(src));
    return result;
}

/* FLD ST(1) + FMUL + FADDP
 * Stack: ST(0)=3.0, ST(1)=2.0.
 * FLD ST(1) pushes 2.0 -> ST(0)=2, ST(1)=3, ST(2)=2.
 * FMUL ST(0),ST(1): ST(0) = 2*3 = 6.
 * FADDP ST(1): ST(1) = 3+6 = 9, pop -> ST(0)=9, ST(1)=2.
 * Result: 9.0.
 */
static double test_b_fld_reg_fmul_faddp(void) {
    double result;
    __asm__ volatile(
        "fld1\n fld1\n faddp\n"                /* 2.0 */
        "fld1\n fld1\n faddp\n fld1\n faddp\n" /* 3.0 */
        /* ST(0)=3, ST(1)=2 */
        "fld %%st(1)\n"
        "fmul %%st(1), %%st(0)\n"
        "faddp %%st(0), %%st(1)\n"
        "fstpl %0\n"
        "fstp %%st(0)\n"
        : "=m"(result));
    return result;
}

/* FLD1 + FMUL + FADDP
 * ST(0)=7.0.  FLD1 -> ST(0)=1, ST(1)=7.
 * FMUL: ST(0) = 1*7 = 7.
 * FADDP: ST(1) = 7+7 = 14, pop.
 * Result: 14.0.
 */
static double test_b_fld1_fmul_faddp(void) {
    double result;
    __asm__ volatile(
        "fld1\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n"
        "fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n" /* 7.0 */
        "fld1\n"
        "fmul %%st(1), %%st(0)\n"
        "faddp %%st(0), %%st(1)\n"
        "fstpl %0\n"
        : "=m"(result));
    return result;
}

/* FILD m32 + FMUL + FADDP
 * ST(0)=3.0.  FILD [4] pushes 4.0 -> FMUL: 4*3=12.
 * FADDP: 3+12=15, pop.
 * Result: 15.0.
 */
static double test_b_fild_fmul_faddp(void) {
    int ival = 4;
    double result;
    __asm__ volatile(
        "fld1\n fld1\n faddp\n fld1\n faddp\n" /* 3.0 */
        "fildl %1\n"
        "fmul %%st(1), %%st(0)\n"
        "faddp %%st(0), %%st(1)\n"
        "fstpl %0\n"
        : "=m"(result)
        : "m"(ival));
    return result;
}

/* =========================================================================
 * Section C: Memory operand on FMUL (FLD + FMUL [mem] + ARITHp)
 * ========================================================================= */

/* FLD m64 + FMUL [mem64] + FADDP
 * ST(0)=5.0.  FLD [2.0] -> ST(0)=2, ST(1)=5.
 * FMUL [4.0]: ST(0) = 2*4 = 8.
 * FADDP: ST(1) = 5+8 = 13, pop.
 * Result: 13.0.
 */
static double test_c_fld_fmul_mem64_faddp(void) {
    double src = 2.0;
    double mul_val = 4.0;
    double result;
    __asm__ volatile(
        "fld1\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n" /* 5.0 */
        "fldl %1\n"
        "fmull %2\n"
        "faddp %%st(0), %%st(1)\n"
        "fstpl %0\n"
        : "=m"(result)
        : "m"(src), "m"(mul_val));
    return result;
}

/* FLD m64 + FMUL [mem32] + FADDP
 * ST(0)=5.0.  FLD [2.0] -> ST(0)=2, ST(1)=5.
 * FMULS [4.0f]: ST(0) = 2*4 = 8.
 * FADDP: ST(1) = 5+8 = 13, pop.
 * Result: 13.0.
 */
static double test_c_fld_fmul_mem32_faddp(void) {
    double src = 2.0;
    float mul_val = 4.0f;
    double result;
    __asm__ volatile(
        "fld1\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n" /* 5.0 */
        "fldl %1\n"
        "fmuls %2\n"
        "faddp %%st(0), %%st(1)\n"
        "fstpl %0\n"
        : "=m"(result)
        : "m"(src), "m"(mul_val));
    return result;
}

/* FLD m64 + FMUL [mem64] + FSUBP (GAS fsubp = Intel FSUBRP)
 * ST(0)=5.0.  FLD [2.0] -> ST(0)=2, ST(1)=5.
 * FMUL [4.0]: ST(0) = 8.
 * FSUBRP: ST(1) = ST(0) - ST(1) = 8 - 5 = 3, pop.
 * Result: 3.0.
 */
static double test_c_fld_fmul_mem64_fsubp(void) {
    double src = 2.0;
    double mul_val = 4.0;
    double result;
    __asm__ volatile(
        "fld1\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n" /* 5.0 */
        "fldl %1\n"
        "fmull %2\n"
        "fsubp %%st(0), %%st(1)\n" /* GAS fsubp = Intel FSUBRP */
        "fstpl %0\n"
        : "=m"(result)
        : "m"(src), "m"(mul_val));
    return result;
}

/* =========================================================================
 * Section D: Stack depth preservation and consecutive fusions
 * ========================================================================= */

/* Deeper stack values survive the fusion.
 * ST(0)=4, ST(1)=100.
 * FLD [3.0] + FMUL + FADDP -> ST(0) = 4 + 3*4 = 16, ST(1) = 100.
 */
static void test_d_stack_depth(double* out_result, double* out_remaining) {
    double src = 3.0;
    __asm__ volatile(
        "fld1\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n"
        "fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n"
        /* 10*10 = ... Actually let's just load 100 from memory. */
        :);
    /* Simpler approach: use volatile memory loads */
    volatile double v_100 = 100.0;
    volatile double v_4 = 4.0;
    __asm__ volatile(
        "fldl %2\n"                /* ST(0) = 100 */
        "fldl %3\n"                /* ST(0) = 4, ST(1) = 100 */
        "fldl %4\n"                /* ST(0) = 3, ST(1) = 4, ST(2) = 100 */
        "fmul %%st(1), %%st(0)\n"  /* ST(0) = 12, ST(1) = 4, ST(2) = 100 */
        "faddp %%st(0), %%st(1)\n" /* ST(0) = 16, ST(1) = 100 */
        "fstpl %0\n"               /* result = 16, pop -> ST(0) = 100 */
        "fstpl %1\n"               /* remaining = 100 */
        : "=m"(*out_result), "=m"(*out_remaining)
        : "m"(v_100), "m"(v_4), "m"(src));
}

/* Two consecutive FLD+FMUL+FADDP fusions.
 * First:  ST(0)=4.  FLD[3]+FMUL+FADDP -> 4+3*4=16.
 * Second: ST(0)=16. FLD[2]+FMUL+FADDP -> 16+2*16=48.
 */
static double test_d_consecutive(void) {
    double src1 = 3.0, src2 = 2.0;
    double result;
    __asm__ volatile(
        "fld1\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n" /* 4.0 */
        /* First fusion: FLD+FMUL+FADDP */
        "fldl %1\n"
        "fmul %%st(1), %%st(0)\n"
        "faddp %%st(0), %%st(1)\n"
        /* ST(0) = 16 */
        /* Second fusion: FLD+FMUL+FADDP */
        "fldl %2\n"
        "fmul %%st(1), %%st(0)\n"
        "faddp %%st(0), %%st(1)\n"
        /* ST(0) = 48 */
        "fstpl %0\n"
        : "=m"(result)
        : "m"(src1), "m"(src2));
    return result;
}

/* =========================================================================
 * Section E: Non-FMA fallback (FADD + FMULP should NOT use FMA)
 * ========================================================================= */

/* FLD m64 + FADD ST(0),ST(1) + FMULP ST(1)
 * ST(0)=2.0.  FLD [5.0] -> ST(0)=5, ST(1)=2.
 * FADD: ST(0) = 5+2 = 7.
 * FMULP: ST(1) = 2*7 = 14, pop.
 * Result: 14.0.
 * This should NOT use the FMA path (arith1=ADD, arith2=MUL).
 */
static double test_e_fld_fadd_fmulp(void) {
    double src = 5.0;
    double result;
    __asm__ volatile(
        "fld1\n fld1\n faddp\n" /* 2.0 */
        "fldl %1\n"
        "fadd %%st(1), %%st(0)\n"
        "fmulp %%st(0), %%st(1)\n"
        "fstpl %0\n"
        : "=m"(result)
        : "m"(src));
    return result;
}

/* FLD m64 + FMUL ST(0),ST(1) + FMULP ST(1) (both multiply — not FMA)
 * ST(0)=2.0.  FLD [5.0] -> ST(0)=5, ST(1)=2.
 * FMUL: ST(0) = 5*2 = 10.
 * FMULP: ST(1) = 2*10 = 20, pop.
 * Result: 20.0.
 */
static double test_e_fld_fmul_fmulp(void) {
    double src = 5.0;
    double result;
    __asm__ volatile(
        "fld1\n fld1\n faddp\n" /* 2.0 */
        "fldl %1\n"
        "fmul %%st(1), %%st(0)\n"
        "fmulp %%st(0), %%st(1)\n"
        "fstpl %0\n"
        : "=m"(result)
        : "m"(src));
    return result;
}

/* FLD m64 + FMUL ST(0),ST(1) + FDIVP ST(1) (mul+div — not FMA)
 * ST(0)=6.0.  FLD [2.0] -> ST(0)=2, ST(1)=6.
 * FMUL: ST(0) = 2*6 = 12.
 * GAS fdivp = Intel FDIVRP: ST(1) = ST(0) / ST(1) = 12 / 6 = 2, pop.
 * Result: 2.0.
 */
static double test_e_fld_fmul_fdivp(void) {
    double src = 2.0;
    double result;
    __asm__ volatile(
        "fld1\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n fld1\n faddp\n" /* 6.0
                                                                                             */
        "fldl %1\n"
        "fmul %%st(1), %%st(0)\n"
        "fdivp %%st(0), %%st(1)\n" /* GAS fdivp = Intel FDIVRP */
        "fstpl %0\n"
        : "=m"(result)
        : "m"(src));
    return result;
}

/* =========================================================================
 * Section F: Larger values / precision check
 * ========================================================================= */

/* FLD m64 + FMUL + FADDP with values that exercise precision.
 * ST(0)=1.5.  FLD [0.1] -> FMUL: 0.1*1.5 = 0.15.  FADDP: 1.5+0.15 = 1.65.
 * Bit-exact: result must match double precision 1.5 + 0.1*1.5.
 */
static double test_f_precision(void) {
    volatile double base = 1.5;
    double src = 0.1;
    double result;
    __asm__ volatile(
        "fldl %1\n"                /* ST(0) = 1.5 */
        "fldl %2\n"                /* ST(0) = 0.1, ST(1) = 1.5 */
        "fmul %%st(1), %%st(0)\n"  /* ST(0) = 0.15, ST(1) = 1.5 */
        "faddp %%st(0), %%st(1)\n" /* ST(0) = 1.65 */
        "fstpl %0\n"
        : "=m"(result)
        : "m"(base), "m"(src));
    return result;
}

int main(void) {
    printf("=== Section A: FLD m64 + FMUL reg + FADDP/FSUBP/FSUBRP ===\n");
    check_f64("A1 FLD m64+FMUL+FADDP:  4+(3*4)=16", test_a_fld_m64_fmul_faddp(), 16.0);
    check_f64("A2 FLD m64+FMUL+FSUBP:  (3*4)-4=8", test_a_fld_m64_fmul_fsubp(), 8.0);
    check_f64("A3 FLD m64+FMUL+FSUBRP: 4-(3*4)=-8", test_a_fld_m64_fmul_fsubrp(), -8.0);

    printf("\n=== Section B: FLD source variants ===\n");
    check_f64("B1 FLD m32+FMUL+FADDP:   5+(2*5)=15", test_b_fld_m32_fmul_faddp(), 15.0);
    check_f64("B2 FLD ST(1)+FMUL+FADDP: 3+(2*3)=9", test_b_fld_reg_fmul_faddp(), 9.0);
    check_f64("B3 FLD1+FMUL+FADDP:      7+(1*7)=14", test_b_fld1_fmul_faddp(), 14.0);
    check_f64("B4 FILD m32+FMUL+FADDP:  3+(4*3)=15", test_b_fild_fmul_faddp(), 15.0);

    printf("\n=== Section C: FMUL memory operand ===\n");
    check_f64("C1 FLD m64+FMUL[m64]+FADDP: 5+(2*4)=13", test_c_fld_fmul_mem64_faddp(), 13.0);
    check_f64("C2 FLD m64+FMUL[m32]+FADDP: 5+(2*4)=13", test_c_fld_fmul_mem32_faddp(), 13.0);
    check_f64("C3 FLD m64+FMUL[m64]+FSUBP: (2*4)-5=3", test_c_fld_fmul_mem64_fsubp(), 3.0);

    printf("\n=== Section D: Stack depth & consecutive fusions ===\n");
    {
        double result, remaining;
        test_d_stack_depth(&result, &remaining);
        check_f64("D1 stack: result 4+(3*4)=16", result, 16.0);
        check_f64("D2 stack: remaining ST(1)=100", remaining, 100.0);
    }
    check_f64("D3 consecutive: 4+3*4=16, then 16+2*16=48", test_d_consecutive(), 48.0);

    printf("\n=== Section E: Non-FMA fallback (still correct) ===\n");
    check_f64("E1 FLD+FADD+FMULP:  2*(5+2)=14", test_e_fld_fadd_fmulp(), 14.0);
    check_f64("E2 FLD+FMUL+FMULP:  2*(5*2)=20", test_e_fld_fmul_fmulp(), 20.0);
    check_f64("E3 FLD+FMUL+FDIVP:  (2*6)/6=2", test_e_fld_fmul_fdivp(), 2.0);

    printf("\n=== Section F: Precision ===\n");
    check_f64("F1 FLD+FMUL+FADDP: 1.5+(0.1*1.5)=1.65", test_f_precision(), 1.65);

    printf("\n%s  (%d failure%s)\n", failures == 0 ? "ALL PASS" : "SOME FAILURES", failures,
           failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
