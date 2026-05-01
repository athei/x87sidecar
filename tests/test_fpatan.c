/*
 * test_fpatan.c — FPATAN inline polynomial approximation.
 *
 * x86 fpatan: ST(0) = atan2(ST(1), ST(0)), pop.
 * Inline asm: fld y; fld x; fpatan; fstpl r — leaves atan2(y, x) in r.
 *
 * The JIT now emits a port of optimized-routines' AdvSIMD atan2
 * (order-19 polynomial, ~2 ULP).  Native Rosetta uses x87 80-bit
 * fpatan and likewise differs from libm in the low bits, so this
 * test compares with a small ULP tolerance.
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

static double do_fpatan(double y, double x) {
    double r;
    __asm__ volatile(
        "fldl  %1\n\t"
        "fldl  %2\n\t"
        "fpatan\n\t"
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(y), "m"(x)
        : "st");
    return r;
}

int main(void) {
    check_ulp("fpatan(0, 1)",       do_fpatan(0.0, 1.0),    atan2(0.0, 1.0));
    check_ulp("fpatan(1, 0)",       do_fpatan(1.0, 0.0),    atan2(1.0, 0.0));
    check_ulp("fpatan(1, 1)",       do_fpatan(1.0, 1.0),    atan2(1.0, 1.0));
    check_ulp("fpatan(-1, 0)",      do_fpatan(-1.0, 0.0),   atan2(-1.0, 0.0));
    check_ulp("fpatan(0, -1)",      do_fpatan(0.0, -1.0),   atan2(0.0, -1.0));
    check_ulp("fpatan(-1, -1)",     do_fpatan(-1.0, -1.0),  atan2(-1.0, -1.0));
    check_ulp("fpatan(2, 3)",       do_fpatan(2.0, 3.0),    atan2(2.0, 3.0));
    check_ulp("fpatan(0.5, 0.5)",   do_fpatan(0.5, 0.5),    atan2(0.5, 0.5));
    check_ulp("fpatan(-3, 4)",      do_fpatan(-3.0, 4.0),   atan2(-3.0, 4.0));
    check_ulp("fpatan(1.5, 2.5)",   do_fpatan(1.5, 2.5),    atan2(1.5, 2.5));

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
