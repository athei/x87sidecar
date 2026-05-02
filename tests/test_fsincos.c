/*
 * test_fsincos.c — FSINCOS inline polynomial approximation.
 *
 * x86 fsincos: ST(0) := sin(old ST(0)), then push cos(old ST(0)) → new ST(0).
 * After: sin in ST(1), cos in ST(0).
 *
 * The JIT now emits inline ARM64 polynomial sin and cos sharing the
 * input load and constants pointer; results match libm within a few
 * ULP rather than bit-exactly.  Native x87 80-bit fsincos likewise
 * differs from libm in the low bits.
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

static void do_fsincos(double v, double* out_sin, double* out_cos) {
    __asm__ volatile(
        "fldl  %2\n\t"
        "fsincos\n\t"
        "fstpl %1\n\t" /* cos at ST(0) → out_cos */
        "fstpl %0\n\t" /* sin at new ST(0) → out_sin */
        : "=m"(*out_sin), "=m"(*out_cos)
        : "m"(v)
        : "st");
}

/* fsincos must clear C2.  See test_fsin.c. */
static uint16_t do_fsincos_sw_after(double v) {
    uint16_t sw;
    double s, c;
    __asm__ volatile(
        "fldl  %3\n\t"
        "fxam\n\t"
        "fsincos\n\t"
        "fnstsw %%ax\n\t"
        "movw  %%ax, %0\n\t"
        "fstpl %2\n\t"
        "fstpl %1\n\t"
        : "=m"(sw), "=m"(s), "=m"(c)
        : "m"(v)
        : "ax", "st");
    (void)s;
    (void)c;
    return sw;
}

static void check_c2_clear(const char* name, uint16_t sw) {
    if ((sw & 0x0400U) == 0U) {
        printf("PASS  %-40s  sw=0x%04x  C2=0\n", name, (unsigned)sw);
    } else {
        printf("FAIL  %-40s  sw=0x%04x  C2=1 (fsincos must clear C2)\n", name, (unsigned)sw);
        failures++;
    }
}

int main(void) {
    double s, c;

    do_fsincos(0.0, &s, &c);
    check_ulp("fsincos(0).sin", s, sin(0.0));
    check_ulp("fsincos(0).cos", c, cos(0.0));

    do_fsincos(1.0, &s, &c);
    check_ulp("fsincos(1).sin", s, sin(1.0));
    check_ulp("fsincos(1).cos", c, cos(1.0));

    do_fsincos(-1.0, &s, &c);
    check_ulp("fsincos(-1).sin", s, sin(-1.0));
    check_ulp("fsincos(-1).cos", c, cos(-1.0));

    do_fsincos(0.5, &s, &c);
    check_ulp("fsincos(0.5).sin", s, sin(0.5));
    check_ulp("fsincos(0.5).cos", c, cos(0.5));

    /* Avoid M_PI/2 — cos near zero hits the same catastrophic
       cancellation regime as test_fcos. */

    check_c2_clear("fsincos(1.0) clears C2", do_fsincos_sw_after(1.0));
    check_c2_clear("fsincos(0.5) clears C2", do_fsincos_sw_after(0.5));
    check_c2_clear("fsincos(-1.0) clears C2", do_fsincos_sw_after(-1.0));

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
