/*
 * test_f2xm1.c — F2XM1 inline polynomial approximation.
 *
 * x87 F2XM1 computes 2^ST(0) - 1, defined for |ST(0)| <= 1.  The JIT
 * emits a port of optimized-routines' AdvSIMD exp2m1 (table-based,
 * worst-case ~3 ULP).  Native Rosetta uses x87 80-bit f2xm1 which also
 * differs from libm exp2(x)-1 in the low bits — ULP-tolerant compare
 * covers both paths.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MAX_ULP 4

static int failures = 0;

static int check_ulp(const char *name, double got, double expected) {
    uint64_t g, e;
    memcpy(&g, &got, sizeof(g));
    memcpy(&e, &expected, sizeof(e));
    if (g == e) {
        printf("PASS  %-40s  got=0x%016llx (%.17g)\n", name,
               (unsigned long long)g, got);
        return 1;
    }
    if (isnan(got) && isnan(expected)) {
        printf("PASS  %-40s  NaN (both)\n", name);
        return 1;
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
        printf("PASS  %-40s  got=0x%016llx (%.17g) [ulp=%llu]\n", name,
               (unsigned long long)g, got, (unsigned long long)ulp_delta);
        return 1;
    }
    printf("FAIL  %-40s  got=0x%016llx (%.17g)  expected=0x%016llx (%.17g)  ulp=%llu\n",
           name, (unsigned long long)g, got, (unsigned long long)e, expected,
           (unsigned long long)ulp_delta);
    failures++;
    return 0;
}

static double do_f2xm1(double v) {
    double r;
    __asm__ volatile(
        "fldl  %1\n\t"
        "f2xm1\n\t"
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(v)
        : "st");
    return r;
}

int main(void) {
    check_ulp("f2xm1(0.0)",        do_f2xm1(0.0),        exp2(0.0) - 1.0);
    check_ulp("f2xm1(1.0)",        do_f2xm1(1.0),        exp2(1.0) - 1.0);
    check_ulp("f2xm1(-1.0)",       do_f2xm1(-1.0),       exp2(-1.0) - 1.0);
    check_ulp("f2xm1(0.5)",        do_f2xm1(0.5),        exp2(0.5) - 1.0);
    check_ulp("f2xm1(-0.5)",       do_f2xm1(-0.5),       exp2(-0.5) - 1.0);
    check_ulp("f2xm1(0.25)",       do_f2xm1(0.25),       exp2(0.25) - 1.0);
    check_ulp("f2xm1(-0.25)",      do_f2xm1(-0.25),      exp2(-0.25) - 1.0);
    check_ulp("f2xm1(0.1)",        do_f2xm1(0.1),        exp2(0.1) - 1.0);
    check_ulp("f2xm1(-0.1)",       do_f2xm1(-0.1),       exp2(-0.1) - 1.0);

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
