/*
 * test_fld_arith.c — Tests for the new 2-op FLD + non-popping ARITH mem
 * fusion (`fld_arith`), which fires when the run does NOT continue with an
 * FSTP (otherwise the 3-op `fld_arith_fstp` would absorb everything).
 *
 * Each test case puts a non-x87 instruction (or a trailing pop in a separate
 * run) after the fld+arith pair so the pair is the natural 2-op contiguous
 * x87 run.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void check_d(const char* name, double got, double expected) {
    uint64_t a;
    uint64_t b;
    memcpy(&a, &got, 8);
    memcpy(&b, &expected, 8);
    if (a != b) {
        printf("FAIL  %-55s  got=%.17g  expected=%.17g\n", name, got, expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

static void check_f(const char* name, float got, float expected) {
    uint32_t a;
    uint32_t b;
    memcpy(&a, &got, 4);
    memcpy(&b, &expected, 4);
    if (a != b) {
        printf("FAIL  %-55s  got=%.9g  expected=%.9g\n", name, (double)got, (double)expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

/* fld m64 + fadd m64.  Force a non-x87 instruction (a register move) between
 * the pair and the eventual fstp so the pair is a contiguous 2-op x87 run.
 * The trailing fstp is in the next x87 run.
 */
static double fld_fadd_m64(double a, double b) {
    double r;
    __asm__ volatile(
        "fldl %1\n\t"
        "faddl %2\n\t"
        "nop\n\t" /* break the contiguous x87 run */
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(a), "m"(b));
    return r;
}

static float fld_fadd_m32(float a, float b) {
    float r;
    __asm__ volatile(
        "flds %1\n\t"
        "fadds %2\n\t"
        "nop\n\t"
        "fstps %0\n\t"
        : "=m"(r)
        : "m"(a), "m"(b));
    return r;
}

static float fld_fmul_m32(float a, float b) {
    float r;
    __asm__ volatile(
        "flds %1\n\t"
        "fmuls %2\n\t"
        "nop\n\t"
        "fstps %0\n\t"
        : "=m"(r)
        : "m"(a), "m"(b));
    return r;
}

static float fld_fsub_m32(float a, float b) {
    float r;
    __asm__ volatile(
        "flds %1\n\t"
        "fsubs %2\n\t"
        "nop\n\t"
        "fstps %0\n\t"
        : "=m"(r)
        : "m"(a), "m"(b));
    return r;
}

static float fld_fsubr_m32(float a, float b) {
    float r;
    __asm__ volatile(
        "flds %1\n\t"
        "fsubrs %2\n\t"
        "nop\n\t"
        "fstps %0\n\t"
        : "=m"(r)
        : "m"(a), "m"(b));
    return r;
}

static float fld_fdiv_m32(float a, float b) {
    float r;
    __asm__ volatile(
        "flds %1\n\t"
        "fdivs %2\n\t"
        "nop\n\t"
        "fstps %0\n\t"
        : "=m"(r)
        : "m"(a), "m"(b));
    return r;
}

static float fld_fdivr_m32(float a, float b) {
    float r;
    __asm__ volatile(
        "flds %1\n\t"
        "fdivrs %2\n\t"
        "nop\n\t"
        "fstps %0\n\t"
        : "=m"(r)
        : "m"(a), "m"(b));
    return r;
}

/* Stack-depth correctness: fld+fadd should leave one extra value on the
 * stack relative to before, with the original ST(0) preserved as the new
 * ST(1).
 */
static void fld_fadd_stack_check(double under, float a, float b, double* under_out,
                                 float* result_out) {
    __asm__ volatile(
        "fldl %2\n\t"  /* push 'under' → ST(0)=under */
        "flds %3\n\t"  /* push a       → ST(0)=a, ST(1)=under */
        "fadds %4\n\t" /* ST(0) = a + b */
        "nop\n\t"
        "fstps %1\n\t" /* store result, pop → ST(0)=under */
        "fstpl %0\n\t" /* store under, pop */
        : "=m"(*under_out), "=m"(*result_out)
        : "m"(under), "m"(a), "m"(b)
        : "memory");
}

int main(void) {
    printf("=== fld+fadd (m64) ===\n");
    check_d("3.0 + 4.5", fld_fadd_m64(3.0, 4.5), 7.5);
    check_d("-1.0 + 2.0", fld_fadd_m64(-1.0, 2.0), 1.0);

    printf("\n=== fld+fadd (m32) ===\n");
    check_f("1.5f + 2.5f", fld_fadd_m32(1.5f, 2.5f), 4.0f);

    printf("\n=== fld+fmul (m32) ===\n");
    check_f("3.0f * 4.0f", fld_fmul_m32(3.0f, 4.0f), 12.0f);
    check_f("0.5f * 0.5f", fld_fmul_m32(0.5f, 0.5f), 0.25f);

    printf("\n=== fld+fsub / fsubr (m32) ===\n");
    check_f("fsub: 10 - 3 = 7", fld_fsub_m32(10.0f, 3.0f), 7.0f);
    check_f("fsubr: 3 - 10 = -7", fld_fsubr_m32(10.0f, 3.0f), -7.0f);

    printf("\n=== fld+fdiv / fdivr (m32) ===\n");
    check_f("fdiv: 12 / 4 = 3", fld_fdiv_m32(12.0f, 4.0f), 3.0f);
    check_f("fdivr: 4 / 12 = 0.333...", fld_fdivr_m32(12.0f, 4.0f), 4.0f / 12.0f);

    printf("\n=== Stack-depth correctness ===\n");
    {
        double under;
        float result;
        fld_fadd_stack_check(99.5, 1.0f, 2.0f, &under, &result);
        check_d("under preserved", under, 99.5);
        check_f("result = a+b = 3.0", result, 3.0f);
    }

    printf("\n%s  (%d failure%s)\n", failures == 0 ? "ALL PASS" : "SOME FAILURES", failures,
           failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
