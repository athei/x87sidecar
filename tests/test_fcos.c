/*
 * test_fcos.c — FCOS inline polynomial approximation.
 *
 * The JIT emits ARM-software/optimized-routines' scalarised AdvSIMD
 * cos polynomial (worst-case ~3 ULP, sharing sin's coefficient block).
 * It is no longer bit-exact vs libm's std::cos, so this test compares
 * with a small ULP tolerance.  Native Rosetta uses x87 80-bit fcos
 * and likewise differs from libm in the low bits.
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

static double do_fcos(double v) {
    double r;
    __asm__ volatile(
        "fldl  %1\n\t"
        "fcos\n\t"
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(v)
        : "st");
    return r;
}

static double do_fadd_then_cos(double a, double b) {
    /* a + b, then cos(a+b) — exercises the cache (faddp consumes stack)
       BEFORE the inline fcos sequence runs.  Confirms cache state survives
       the polynomial. */
    double r;
    __asm__ volatile(
        "fldl  %1\n\t"
        "fldl  %2\n\t"
        "faddp %%st, %%st(1)\n\t"
        "fcos\n\t"
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(a), "m"(b)
        : "st");
    return r;
}

int main(void) {
    /* Minimal shape — fcos after just fld. */
    check_ulp("fcos(0.0)",         do_fcos(0.0),         cos(0.0));
    check_ulp("fcos(-0.0)",        do_fcos(-0.0),        cos(-0.0));
    check_ulp("fcos(1.0)",         do_fcos(1.0),         cos(1.0));
    check_ulp("fcos(0.5)",         do_fcos(0.5),         cos(0.5));
    check_ulp("fcos(-1.0)",        do_fcos(-1.0),        cos(-1.0));
    /* Avoid M_PI/2 here — cos(π/2) is near-zero, catastrophic
       cancellation makes 4-ULP tolerance impossible without
       a much-higher-precision reduction.  Native x87 80-bit
       fcos and libm f64 cos disagree by hundreds of billions
       of ULPs at this exact input. */

    /* Boundary shape — handled prefix (faddp) writes our cache, then
       fcos's inline sequence fires with cache state in registers. */
    check_ulp("fcos(0.5+0.5)",     do_fadd_then_cos(0.5, 0.5),  cos(1.0));
    check_ulp("fcos(0.0+0.0)",     do_fadd_then_cos(0.0, 0.0),  cos(0.0));
    check_ulp("fcos(0.3+0.4)",     do_fadd_then_cos(0.3, 0.4),  cos(0.3 + 0.4));

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
