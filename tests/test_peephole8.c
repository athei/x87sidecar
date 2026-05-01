/*
 * test_peephole8.c -- validate arith_fstp fusion (OPT-F14)
 *
 * Build:  clang -arch x86_64 -O0 -g -o test_peephole8 test_peephole8.c
 *
 * Tests non-popping arithmetic (FMUL/FADD/FSUB/FDIV/FSUBR/FDIVR) followed
 * by FSTP to memory.  The fusion skips the intermediate stack writeback and
 * stores the arithmetic result directly to the FSTP destination.
 *
 * Sections:
 *   A: Register-register forms  (ARITH ST(0), ST(i) + FSTP)
 *   B: Memory-operand forms     (ARITH [mem] + FSTP)
 *   C: FSTP m32 (f32 truncation paths)
 *   D: FSUBR / FDIVR reversed-operand forms
 *   E: Stack state verification (ST(0) consumed after fusion)
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
        printf("FAIL  %-60s  got=%.15g  expected=%.15g\n", name, got, expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

static void check_f32(const char* name, float got, float expected) {
    if (as_u32(got) != as_u32(expected)) {
        printf("FAIL  %-60s  got=%.10g (0x%08x)  expected=%.10g (0x%08x)\n", name, got, as_u32(got),
               expected, as_u32(expected));
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

/* ========================================================================= */
/* Section A: Register-register forms -- ARITH ST(0),ST(i) + FSTP m64       */
/* ========================================================================= */

/* FMUL ST(0),ST(1) + FSTP [m64]:  3.0 * 4.0 = 12.0 */
static void test_fmul_reg_fstp_m64(void) {
    volatile double a = 3.0, b = 4.0;
    volatile double r = 0.0;
    __asm__ volatile(
        "fldl %1\n\t"            /* ST(0)=b=4 */
        "fldl %2\n\t"            /* ST(0)=a=3, ST(1)=b=4 */
        "fmul %%st(1), %%st\n\t" /* ST(0)=3*4=12, ST(1)=4 (arith_fstp target) */
        "fstpl %0\n\t"           /* store 12 to r, pop */
        "fstp %%st(0)\n"         /* clean up ST(0)=4 */
        : "=m"(r)
        : "m"(b), "m"(a));
    check_f64("A.1 FMUL ST(0),ST(1) + FSTP m64", r, 12.0);
}

/* FADD ST(0),ST(1) + FSTP [m64]:  3.0 + 4.0 = 7.0 */
static void test_fadd_reg_fstp_m64(void) {
    volatile double a = 3.0, b = 4.0;
    volatile double r = 0.0;
    __asm__ volatile(
        "fldl %1\n\t"
        "fldl %2\n\t"
        "fadd %%st(1), %%st\n\t"
        "fstpl %0\n\t"
        "fstp %%st(0)\n"
        : "=m"(r)
        : "m"(b), "m"(a));
    check_f64("A.2 FADD ST(0),ST(1) + FSTP m64", r, 7.0);
}

/* FSUB ST(0),ST(1) + FSTP [m64]:  3.0 - 4.0 = -1.0 */
static void test_fsub_reg_fstp_m64(void) {
    volatile double a = 3.0, b = 4.0;
    volatile double r = 0.0;
    __asm__ volatile(
        "fldl %1\n\t"
        "fldl %2\n\t"
        "fsub %%st(1), %%st\n\t"
        "fstpl %0\n\t"
        "fstp %%st(0)\n"
        : "=m"(r)
        : "m"(b), "m"(a));
    check_f64("A.3 FSUB ST(0),ST(1) + FSTP m64", r, -1.0);
}

/* FDIV ST(0),ST(1) + FSTP [m64]:  3.0 / 4.0 = 0.75 */
static void test_fdiv_reg_fstp_m64(void) {
    volatile double a = 3.0, b = 4.0;
    volatile double r = 0.0;
    __asm__ volatile(
        "fldl %1\n\t"
        "fldl %2\n\t"
        "fdiv %%st(1), %%st\n\t"
        "fstpl %0\n\t"
        "fstp %%st(0)\n"
        : "=m"(r)
        : "m"(b), "m"(a));
    check_f64("A.4 FDIV ST(0),ST(1) + FSTP m64", r, 0.75);
}

/* FMUL ST(0),ST(2) + FSTP [m64]: deeper source register */
static void test_fmul_deep_reg_fstp_m64(void) {
    volatile double a = 2.0, b = 3.0, c = 5.0;
    volatile double r = 0.0;
    __asm__ volatile(
        "fldl %1\n\t"            /* ST(0)=c=5 */
        "fldl %2\n\t"            /* ST(0)=b=3, ST(1)=5 */
        "fldl %3\n\t"            /* ST(0)=a=2, ST(1)=3, ST(2)=5 */
        "fmul %%st(2), %%st\n\t" /* ST(0)=2*5=10 */
        "fstpl %0\n\t"           /* store 10, pop */
        "fstp %%st(0)\n\t"
        "fstp %%st(0)\n"
        : "=m"(r)
        : "m"(c), "m"(b), "m"(a));
    check_f64("A.5 FMUL ST(0),ST(2) + FSTP m64 (deep reg)", r, 10.0);
}

/* ========================================================================= */
/* Section B: Memory-operand forms -- ARITH [mem] + FSTP m64                */
/* ========================================================================= */

/* FMUL [m64] + FSTP [m64]:  3.0 * 4.0 = 12.0 */
static void test_fmul_mem64_fstp_m64(void) {
    volatile double a = 3.0, b = 4.0;
    volatile double r = 0.0;
    __asm__ volatile(
        "fldl %1\n\t"  /* ST(0)=a=3 */
        "fmull %2\n\t" /* ST(0)=3*4=12 (arith_fstp target) */
        "fstpl %0\n"
        : "=m"(r)
        : "m"(a), "m"(b));
    check_f64("B.1 FMUL m64 + FSTP m64", r, 12.0);
}

/* FADD [m64] + FSTP [m64]:  3.0 + 4.0 = 7.0 */
static void test_fadd_mem64_fstp_m64(void) {
    volatile double a = 3.0, b = 4.0;
    volatile double r = 0.0;
    __asm__ volatile(
        "fldl %1\n\t"
        "faddl %2\n\t"
        "fstpl %0\n"
        : "=m"(r)
        : "m"(a), "m"(b));
    check_f64("B.2 FADD m64 + FSTP m64", r, 7.0);
}

/* FSUB [m64] + FSTP [m64]:  3.0 - 4.0 = -1.0 */
static void test_fsub_mem64_fstp_m64(void) {
    volatile double a = 3.0, b = 4.0;
    volatile double r = 0.0;
    __asm__ volatile(
        "fldl %1\n\t"
        "fsubl %2\n\t"
        "fstpl %0\n"
        : "=m"(r)
        : "m"(a), "m"(b));
    check_f64("B.3 FSUB m64 + FSTP m64", r, -1.0);
}

/* FDIV [m64] + FSTP [m64]:  3.0 / 4.0 = 0.75 */
static void test_fdiv_mem64_fstp_m64(void) {
    volatile double a = 3.0, b = 4.0;
    volatile double r = 0.0;
    __asm__ volatile(
        "fldl %1\n\t"
        "fdivl %2\n\t"
        "fstpl %0\n"
        : "=m"(r)
        : "m"(a), "m"(b));
    check_f64("B.4 FDIV m64 + FSTP m64", r, 0.75);
}

/* FMUL [m32] + FSTP [m64]:  3.0 * 4.0f = 12.0 */
static void test_fmul_mem32_fstp_m64(void) {
    volatile double a = 3.0;
    volatile float b = 4.0f;
    volatile double r = 0.0;
    __asm__ volatile(
        "fldl %1\n\t"
        "fmuls %2\n\t"
        "fstpl %0\n"
        : "=m"(r)
        : "m"(a), "m"(b));
    check_f64("B.5 FMUL m32 + FSTP m64", r, 12.0);
}

/* ========================================================================= */
/* Section C: FSTP m32 paths (f64 → f32 truncation)                         */
/* ========================================================================= */

/* FMUL [m64] + FSTP [m32]:  3.0 * 4.0 = 12.0f */
static void test_fmul_mem64_fstp_m32(void) {
    volatile double a = 3.0, b = 4.0;
    volatile float r = 0.0f;
    __asm__ volatile(
        "fldl %1\n\t"
        "fmull %2\n\t"
        "fstps %0\n"
        : "=m"(r)
        : "m"(a), "m"(b));
    check_f32("C.1 FMUL m64 + FSTP m32", r, 12.0f);
}

/* FADD [m32] + FSTP [m32]:  3.0f + 4.0f = 7.0f */
static void test_fadd_mem32_fstp_m32(void) {
    volatile double a = 3.0;
    volatile float b = 4.0f;
    volatile float r = 0.0f;
    __asm__ volatile(
        "fldl %1\n\t"
        "fadds %2\n\t"
        "fstps %0\n"
        : "=m"(r)
        : "m"(a), "m"(b));
    check_f32("C.2 FADD m32 + FSTP m32", r, 7.0f);
}

/* FMUL ST(0),ST(1) + FSTP [m32]:  3.0 * 4.0 = 12.0f */
static void test_fmul_reg_fstp_m32(void) {
    volatile double a = 3.0, b = 4.0;
    volatile float r = 0.0f;
    __asm__ volatile(
        "fldl %1\n\t"
        "fldl %2\n\t"
        "fmul %%st(1), %%st\n\t"
        "fstps %0\n\t"
        "fstp %%st(0)\n"
        : "=m"(r)
        : "m"(b), "m"(a));
    check_f32("C.3 FMUL ST(0),ST(1) + FSTP m32", r, 12.0f);
}

/* ========================================================================= */
/* Section D: Reversed forms -- FSUBR / FDIVR                               */
/* ========================================================================= */

/* FSUBR ST(0),ST(1) + FSTP [m64]:  ST(1) - ST(0) = 4.0 - 3.0 = 1.0 */
static void test_fsubr_reg_fstp_m64(void) {
    volatile double a = 3.0, b = 4.0;
    volatile double r = 0.0;
    __asm__ volatile(
        "fldl %1\n\t"             /* ST(0)=b=4 */
        "fldl %2\n\t"             /* ST(0)=a=3, ST(1)=4 */
        "fsubr %%st(1), %%st\n\t" /* ST(0)=4-3=1 */
        "fstpl %0\n\t"
        "fstp %%st(0)\n"
        : "=m"(r)
        : "m"(b), "m"(a));
    check_f64("D.1 FSUBR ST(0),ST(1) + FSTP m64", r, 1.0);
}

/* FDIVR ST(0),ST(1) + FSTP [m64]:  ST(1) / ST(0) = 4.0 / 3.0 */
static void test_fdivr_reg_fstp_m64(void) {
    volatile double a = 3.0, b = 4.0;
    volatile double r = 0.0;
    __asm__ volatile(
        "fldl %1\n\t"
        "fldl %2\n\t"
        "fdivr %%st(1), %%st\n\t"
        "fstpl %0\n\t"
        "fstp %%st(0)\n"
        : "=m"(r)
        : "m"(b), "m"(a));
    check_f64("D.2 FDIVR ST(0),ST(1) + FSTP m64", r, 4.0 / 3.0);
}

/* FDIVR [m32] + FSTP [m32]:  4.0f / 3.0 = 1.333...f */
static void test_fdivr_mem32_fstp_m32(void) {
    volatile double a = 3.0;
    volatile float b = 4.0f;
    volatile float r = 0.0f;
    __asm__ volatile(
        "fldl %1\n\t"   /* ST(0)=a=3.0 */
        "fdivrs %2\n\t" /* ST(0)=4.0/3.0 (reversed) */
        "fstps %0\n"
        : "=m"(r)
        : "m"(a), "m"(b));
    check_f32("D.3 FDIVR m32 + FSTP m32", r, 4.0f / 3.0f);
}

/* ========================================================================= */
/* Section E: Stack state verification                                       */
/* ========================================================================= */

/* Verify that after ARITH+FSTP fusion, exactly one value is popped and
   remaining stack values are correct. */
static void test_stack_state_after_fusion(void) {
    volatile double a = 2.0, b = 3.0, c = 5.0;
    volatile double r_product = 0.0, r_remaining = 0.0;
    __asm__ volatile(
        "fldl %2\n\t"            /* ST(0)=c=5 */
        "fldl %3\n\t"            /* ST(0)=b=3, ST(1)=5 */
        "fldl %4\n\t"            /* ST(0)=a=2, ST(1)=3, ST(2)=5 */
        "fmul %%st(1), %%st\n\t" /* ST(0)=2*3=6, ST(1)=3, ST(2)=5 -- arith_fstp fires */
        "fstpl %0\n\t"           /* store 6, pop -> ST(0)=3, ST(1)=5 */
        "fstpl %1\n\t"           /* store 3 (remaining ST(0)), pop -> ST(0)=5 */
        "fstp %%st(0)\n"         /* clean up */
        : "=m"(r_product), "=m"(r_remaining)
        : "m"(c), "m"(b), "m"(a));
    check_f64("E.1 stack: product", r_product, 6.0);
    check_f64("E.2 stack: remaining ST(0) after pop", r_remaining, 3.0);
}

/* Chain: two arith+fstp fusions in sequence */
static void test_double_arith_fstp_chain(void) {
    volatile double a = 2.0, b = 3.0, c = 7.0;
    volatile double r1 = 0.0, r2 = 0.0;
    __asm__ volatile(
        "fldl %2\n\t"            /* ST(0)=c=7 */
        "fldl %3\n\t"            /* ST(0)=b=3, ST(1)=7 */
        "fldl %4\n\t"            /* ST(0)=a=2, ST(1)=3, ST(2)=7 */
        "fmul %%st(1), %%st\n\t" /* ST(0)=6, ST(1)=3, ST(2)=7 -- fuse #1 */
        "fstpl %0\n\t"           /* r1=6, pop -> ST(0)=3, ST(1)=7 */
        "fadd %%st(1), %%st\n\t" /* ST(0)=3+7=10, ST(1)=7 -- fuse #2 */
        "fstpl %1\n\t"           /* r2=10, pop -> ST(0)=7 */
        "fstp %%st(0)\n"
        : "=m"(r1), "=m"(r2)
        : "m"(c), "m"(b), "m"(a));
    check_f64("E.3 chain: first arith+fstp", r1, 6.0);
    check_f64("E.4 chain: second arith+fstp", r2, 10.0);
}

int main(void) {
    printf("=== arith_fstp fusion tests (OPT-F14) ===\n\n");

    printf("--- Section A: Register-register forms ---\n");
    test_fmul_reg_fstp_m64();
    test_fadd_reg_fstp_m64();
    test_fsub_reg_fstp_m64();
    test_fdiv_reg_fstp_m64();
    test_fmul_deep_reg_fstp_m64();

    printf("\n--- Section B: Memory-operand forms ---\n");
    test_fmul_mem64_fstp_m64();
    test_fadd_mem64_fstp_m64();
    test_fsub_mem64_fstp_m64();
    test_fdiv_mem64_fstp_m64();
    test_fmul_mem32_fstp_m64();

    printf("\n--- Section C: FSTP m32 paths ---\n");
    test_fmul_mem64_fstp_m32();
    test_fadd_mem32_fstp_m32();
    test_fmul_reg_fstp_m32();

    printf("\n--- Section D: Reversed forms ---\n");
    test_fsubr_reg_fstp_m64();
    test_fdivr_reg_fstp_m64();
    test_fdivr_mem32_fstp_m32();

    printf("\n--- Section E: Stack state ---\n");
    test_stack_state_after_fusion();
    test_double_arith_fstp_chain();

    printf("\n%s: %d failures\n", failures ? "FAILED" : "ALL PASSED", failures);
    return failures ? 1 : 0;
}
