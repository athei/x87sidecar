/*
 * test_fyl2x.c — FYL2X inline polynomial approximation.
 *
 * x86 fyl2x: ST(1) := ST(1) * log2(ST(0)); pop ST(0).
 * Inline asm: fld y; fld x; fyl2x; fstpl r — leaves y * log2(x) in r.
 *
 * The JIT now emits a port of optimized-routines' AdvSIMD inline_log2
 * (~2 ULP) followed by FMUL.  Native x87 80-bit fyl2x likewise differs
 * from libm in the low bits, so this test compares with a small ULP
 * tolerance.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_ULP 4

static int failures = 0;

static int check_ulp(const char* name, double got, double expected) {
    uint64_t g, e;
    memcpy(&g, &got, sizeof(g));
    memcpy(&e, &expected, sizeof(e));
    if (g == e) {
        printf("PASS  %-40s  got=0x%016llx (%.17g)\n", name, (unsigned long long)g, got);
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
        printf("PASS  %-40s  got=0x%016llx (%.17g) [ulp=%llu]\n", name, (unsigned long long)g, got,
               (unsigned long long)ulp_delta);
        return 1;
    }
    printf("FAIL  %-40s  got=0x%016llx (%.17g)  expected=0x%016llx (%.17g)  ulp=%llu\n", name,
           (unsigned long long)g, got, (unsigned long long)e, expected,
           (unsigned long long)ulp_delta);
    failures++;
    return 0;
}

static double do_fyl2x(double y, double x) {
    double r;
    __asm__ volatile(
        "fldl  %1\n\t" /* push y → ST(0) */
        "fldl  %2\n\t" /* push x → ST(0); y now ST(1) */
        "fyl2x\n\t"    /* ST(1) = y*log2(x); pop ST(0) */
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(y), "m"(x)
        : "st");
    return r;
}

int main(void) {
    check_ulp("fyl2x(1, 2)", do_fyl2x(1.0, 2.0), 1.0 * log2(2.0));
    check_ulp("fyl2x(2, 2)", do_fyl2x(2.0, 2.0), 2.0 * log2(2.0));
    check_ulp("fyl2x(1, 1)", do_fyl2x(1.0, 1.0), 1.0 * log2(1.0));
    check_ulp("fyl2x(3, 4)", do_fyl2x(3.0, 4.0), 3.0 * log2(4.0));
    check_ulp("fyl2x(1, 4)", do_fyl2x(1.0, 4.0), 1.0 * log2(4.0));
    check_ulp("fyl2x(1, 3)", do_fyl2x(1.0, 3.0), 1.0 * log2(3.0));
    check_ulp("fyl2x(2, 1.5)", do_fyl2x(2.0, 1.5), 2.0 * log2(1.5));
    check_ulp("fyl2x(1, 0.5)", do_fyl2x(1.0, 0.5), 1.0 * log2(0.5));
    check_ulp("fyl2x(1, 0.1)", do_fyl2x(1.0, 0.1), 1.0 * log2(0.1));

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
