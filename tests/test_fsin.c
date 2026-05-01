/*
 * test_fsin.c — FSIN inline polynomial approximation.
 *
 * The JIT now emits ARM-software/optimized-routines' scalarised AdvSIMD
 * sin polynomial (worst-case ~3 ULP).  It is no longer bit-exact vs
 * libm's std::sin, so this test compares with a small ULP tolerance.
 *
 * Native Rosetta uses x87 80-bit fsin and likewise differs from libm in
 * the low bits.  ULP-tolerant comparison covers both paths.
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

    /* Bit-identical — the easy case (covers ±0, exactly representable
       results, etc.). */
    if (g == e) {
        printf("PASS  %-40s  got=0x%016llx (%.17g)\n", name, (unsigned long long)g, got);
        return 1;
    }

    /* NaN handling: any NaN is acceptable as long as the expected value
       is also NaN.  We don't preserve specific NaN payloads. */
    if (isnan(got) && isnan(expected)) {
        printf("PASS  %-40s  NaN (both)\n", name);
        return 1;
    }

    /* ULP delta.  For same-sign operands, |g - e| in two's-complement on
       the bit pattern equals the ULP distance; for opposite-sign across
       zero, sum the magnitudes. */
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

static double do_fsin(double v) {
    double r;
    __asm__ volatile(
        "fldl  %1\n\t"
        "fsin\n\t"
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(v)
        : "st");
    return r;
}

static double do_fadd_then_sin(double a, double b) {
    /* a + b, then sin(a+b) — exercises the cache (faddp consumes stack)
       BEFORE the inline fsin sequence runs.  Confirms cache state survives
       the polynomial. */
    double r;
    __asm__ volatile(
        "fldl  %1\n\t"
        "fldl  %2\n\t"
        "faddp %%st, %%st(1)\n\t"
        "fsin\n\t"
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(a), "m"(b)
        : "st");
    return r;
}

int main(void) {
    /* Minimal shape — fsin after just fld. */
    check_ulp("fsin(0.0)", do_fsin(0.0), sin(0.0));
    check_ulp("fsin(-0.0)", do_fsin(-0.0), sin(-0.0));
    check_ulp("fsin(1.0)", do_fsin(1.0), sin(1.0));
    check_ulp("fsin(0.5)", do_fsin(0.5), sin(0.5));
    check_ulp("fsin(M_PI/2)", do_fsin(M_PI / 2.0), sin(M_PI / 2.0));
    check_ulp("fsin(-1.0)", do_fsin(-1.0), sin(-1.0));

    /* Boundary shape — handled prefix (faddp) writes our cache, then
       fsin's inline sequence fires with cache state in registers. */
    check_ulp("fsin(0.5+0.5)", do_fadd_then_sin(0.5, 0.5), sin(1.0));
    check_ulp("fsin(0.0+0.0)", do_fadd_then_sin(0.0, 0.0), sin(0.0));
    check_ulp("fsin(0.3+0.4)", do_fadd_then_sin(0.3, 0.4), sin(0.3 + 0.4));

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
