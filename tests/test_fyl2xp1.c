/*
 * test_fyl2xp1.c — FYL2XP1 inline polynomial approximation.
 *
 * x86 fyl2xp1: ST(1) := ST(1) * log2(ST(0) + 1.0); pop ST(0).
 * Domain: -1+1/sqrt(2) <= ST(0) <= 1-1/sqrt(2) for full x87 precision,
 * but the inline impl uses simple add-then-log2 which is accurate
 * within a few ULP across the spec range.
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

static double do_fyl2xp1(double y, double x) {
    double r;
    __asm__ volatile(
        "fldl  %1\n\t"   /* push y → ST(0) */
        "fldl  %2\n\t"   /* push x → ST(0); y now ST(1) */
        "fyl2xp1\n\t"    /* ST(1) = y*log2(x+1); pop */
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(y), "m"(x)
        : "st");
    return r;
}

int main(void) {
    check_ulp("fyl2xp1(1, 0)",   do_fyl2xp1(1.0, 0.0),   1.0 * log2(1.0));
    check_ulp("fyl2xp1(2, 0)",   do_fyl2xp1(2.0, 0.0),   2.0 * log2(1.0));
    check_ulp("fyl2xp1(1, 1)",   do_fyl2xp1(1.0, 1.0),   1.0 * log2(2.0));
    check_ulp("fyl2xp1(3, 1)",   do_fyl2xp1(3.0, 1.0),   3.0 * log2(2.0));
    check_ulp("fyl2xp1(1, 3)",   do_fyl2xp1(1.0, 3.0),   1.0 * log2(4.0));
    check_ulp("fyl2xp1(1, 0.1)", do_fyl2xp1(1.0, 0.1),   1.0 * log2(1.1));
    check_ulp("fyl2xp1(1,-0.1)", do_fyl2xp1(1.0, -0.1),  1.0 * log2(0.9));
    check_ulp("fyl2xp1(2, 0.25)",do_fyl2xp1(2.0, 0.25),  2.0 * log2(1.25));

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
