/*
 * test_fptan.c — FPTAN inline polynomial approximation.
 *
 * x86 fptan: ST(0) := tan(ST(0)), push 1.0.  After: ST(0)=1.0, ST(1)=tan(v).
 *
 * The JIT now emits a port of optimized-routines' AdvSIMD tan
 * (order-7 Estrin + double-angle recombination, ~3 ULP).  Native
 * Rosetta uses x87 80-bit fptan; both paths' f64 outputs differ from
 * libm's tan in the low bits.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MAX_ULP 8

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

static void do_fptan(double v, double *out_tan, double *out_one) {
    __asm__ volatile(
        "fldl  %2\n\t"
        "fptan\n\t"
        "fstpl %1\n\t"   /* 1.0 at ST(0) → out_one */
        "fstpl %0\n\t"   /* tan at new ST(0) → out_tan */
        : "=m"(*out_tan), "=m"(*out_one)
        : "m"(v)
        : "st");
}

int main(void) {
    double t, one;

    do_fptan(0.0, &t, &one);
    check_ulp("fptan(0).tan",     t,   tan(0.0));
    check_ulp("fptan(0).one",     one, 1.0);

    do_fptan(-0.0, &t, &one);
    check_ulp("fptan(-0).tan",    t,   tan(-0.0));
    check_ulp("fptan(-0).one",    one, 1.0);

    do_fptan(1.0, &t, &one);
    check_ulp("fptan(1).tan",     t,   tan(1.0));
    check_ulp("fptan(1).one",     one, 1.0);

    do_fptan(0.5, &t, &one);
    check_ulp("fptan(0.5).tan",   t,   tan(0.5));
    check_ulp("fptan(0.5).one",   one, 1.0);

    do_fptan(-1.0, &t, &one);
    check_ulp("fptan(-1).tan",    t,   tan(-1.0));
    check_ulp("fptan(-1).one",    one, 1.0);

    do_fptan(0.785, &t, &one);   /* close to π/4 */
    check_ulp("fptan(0.785).tan", t,   tan(0.785));

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
