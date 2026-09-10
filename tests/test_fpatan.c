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

// Zero results: check_ulp treats +0 and -0 as equal (0 ULP apart); IEEE
// atan2 distinguishes them, so compare the sign bit too.
static int check_signed_zero(const char* name, double got, double expected) {
    if (signbit(got) != signbit(expected)) {
        printf("FAIL  %-40s  got=%s0 expected=%s0 (sign of zero)\n", name, signbit(got) ? "-" : "+",
               signbit(expected) ? "-" : "+");
        failures++;
        return 0;
    }
    return check_ulp(name, got, expected);
}

static int check_nan(const char* name, double got) {
    uint64_t g;
    memcpy(&g, &got, sizeof(g));
    if (isnan(got)) {
        printf("PASS  %-40s  NaN (0x%016llx)\n", name, (unsigned long long)g);
        return 1;
    }
    printf("FAIL  %-40s  got=0x%016llx (%.17g)  expected NaN\n", name, (unsigned long long)g, got);
    failures++;
    return 0;
}

// The MSVC CRT acos() body (msssoft.m3d, CoD2SP_s.exe, mss32.dll all carry
// it): atan2(sqrt((1+d)(1-d)), d).
static double do_crt_acos(double d) {
    double r;
    __asm__ volatile(
        "fldl  %1\n\t"
        "fld1\n\t"
        "fadd  %%st(1), %%st\n\t"
        "fld1\n\t"
        "fsub  %%st(2), %%st\n\t"
        "fmulp\n\t"
        "fsqrt\n\t"
        "fxch\n\t"
        "fpatan\n\t"
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(d)
        : "st", "st(1)", "st(2)");
    return r;
}

int main(void) {
    check_ulp("fpatan(0, 1)", do_fpatan(0.0, 1.0), atan2(0.0, 1.0));
    check_ulp("fpatan(1, 0)", do_fpatan(1.0, 0.0), atan2(1.0, 0.0));
    check_ulp("fpatan(1, 1)", do_fpatan(1.0, 1.0), atan2(1.0, 1.0));
    check_ulp("fpatan(-1, 0)", do_fpatan(-1.0, 0.0), atan2(-1.0, 0.0));
    check_ulp("fpatan(0, -1)", do_fpatan(0.0, -1.0), atan2(0.0, -1.0));
    check_ulp("fpatan(-1, -1)", do_fpatan(-1.0, -1.0), atan2(-1.0, -1.0));
    check_ulp("fpatan(2, 3)", do_fpatan(2.0, 3.0), atan2(2.0, 3.0));
    check_ulp("fpatan(0.5, 0.5)", do_fpatan(0.5, 0.5), atan2(0.5, 0.5));
    check_ulp("fpatan(-3, 4)", do_fpatan(-3.0, 4.0), atan2(-3.0, 4.0));
    check_ulp("fpatan(1.5, 2.5)", do_fpatan(1.5, 2.5), atan2(1.5, 2.5));

    // Signed zeros and infinities.  The AdvSIMD algorithm's "x < 0" test
    // must be the sign bit: an FCMP against zero misses -0.0 and the
    // sign_xy XOR then flips the quadrant — atan2(1, -0.0) came out as
    // -pi/2.  The CRT acos() hits this for every d = -0.0 (Miles'
    // msssoft.m3d pans a source straight ahead with acos(dot)/pi and
    // produced a negative channel volume, CoD2 issue #23).  Both-zero
    // inputs must not divide 0/0 into NaN.  Zero results are compared
    // with their sign.
    check_ulp("fpatan(1, -0.0)", do_fpatan(1.0, -0.0), atan2(1.0, -0.0));
    check_ulp("fpatan(-1, -0.0)", do_fpatan(-1.0, -0.0), atan2(-1.0, -0.0));
    check_ulp("fpatan(0.5, -0.0)", do_fpatan(0.5, -0.0), atan2(0.5, -0.0));
    check_ulp("fpatan(0.0, -0.0)", do_fpatan(0.0, -0.0), atan2(0.0, -0.0));
    check_ulp("fpatan(-0.0, -0.0)", do_fpatan(-0.0, -0.0), atan2(-0.0, -0.0));
    check_ulp("fpatan(-0.0, -1)", do_fpatan(-0.0, -1.0), atan2(-0.0, -1.0));
    check_signed_zero("fpatan(0.0, 0.0)", do_fpatan(0.0, 0.0), atan2(0.0, 0.0));
    check_signed_zero("fpatan(-0.0, 0.0)", do_fpatan(-0.0, 0.0), atan2(-0.0, 0.0));
    check_signed_zero("fpatan(-0.0, 1)", do_fpatan(-0.0, 1.0), atan2(-0.0, 1.0));
    check_ulp("fpatan(1, -inf)", do_fpatan(1.0, -INFINITY), atan2(1.0, -INFINITY));
    check_ulp("fpatan(-1, -inf)", do_fpatan(-1.0, -INFINITY), atan2(-1.0, -INFINITY));
    check_signed_zero("fpatan(1, +inf)", do_fpatan(1.0, INFINITY), atan2(1.0, INFINITY));
    check_ulp("fpatan(+inf, 1)", do_fpatan(INFINITY, 1.0), atan2(INFINITY, 1.0));
    check_ulp("fpatan(-inf, -1)", do_fpatan(-INFINITY, -1.0), atan2(-INFINITY, -1.0));
    // NaN must propagate through the both-zero guard: y = NaN with x = ±0
    // leaves num = NaN and den = 0, and z must stay NaN, not become 0.
    check_nan("fpatan(NaN, +0.0)", do_fpatan(NAN, 0.0));
    check_nan("fpatan(NaN, -0.0)", do_fpatan(NAN, -0.0));
    check_nan("fpatan(-NaN, +0.0)", do_fpatan(-NAN, 0.0));
    check_nan("fpatan(-NaN, -0.0)", do_fpatan(-NAN, -0.0));
    check_nan("fpatan(+0.0, NaN)", do_fpatan(0.0, NAN));
    check_nan("fpatan(-0.0, NaN)", do_fpatan(-0.0, NAN));
    check_nan("fpatan(NaN, NaN)", do_fpatan(NAN, NAN));
    check_nan("fpatan(1, NaN)", do_fpatan(1.0, NAN));
    check_nan("fpatan(NaN, 1)", do_fpatan(NAN, 1.0));
    check_ulp("acos(-0.0) via CRT sequence", do_crt_acos(-0.0), acos(-0.0));
    check_ulp("acos(+0.0) via CRT sequence", do_crt_acos(0.0), acos(0.0));

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
