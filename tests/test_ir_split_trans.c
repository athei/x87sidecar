/*
 * test_ir_split_trans.c — Transcendental inside a high-pressure run.
 *
 * The inlined fsin polynomial spikes ~7 FPRs; a run that also holds
 * several live products historically refused on FprPressure and fell to
 * per-op emission.  With pressure splitting the run cuts just before the
 * transcendental (the suffix starts nearly empty, so spike + 1 fits even
 * the minimum pool).  Numerics must be identical on every path; sin uses
 * a minimax polynomial, so compare with ULP tolerance like test_fsin.
 *
 * Build: clang -arch x86_64 -O0 -o test_ir_split_trans test_ir_split_trans.c -lm
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MAX_ULP 4

static int failures = 0;

static int check_ulp(const char* name, double got, double expected) {
    uint64_t g, e;
    memcpy(&g, &got, sizeof(g));
    memcpy(&e, &expected, sizeof(e));
    if (g == e) {
        printf("PASS  %s\n", name);
        return 1;
    }
    if (isnan(got) && isnan(expected)) {
        printf("PASS  %s  NaN (both)\n", name);
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
        printf("PASS  %s  [ulp=%llu]\n", name, (unsigned long long)ulp_delta);
        return 1;
    }
    printf("FAIL  %-45s  got=%.17g  expected=%.17g  ulp=%llu\n", name, got, expected,
           (unsigned long long)ulp_delta);
    failures++;
    return 0;
}

/* Products held live across an fsin: pressure peak at the transcendental. */
static double products_then_fsin(const double* a) {
    double out;
    __asm__ volatile(
        "fldl   (%1)\n\t"
        "fmull  24(%1)\n\t"
        "fldl   8(%1)\n\t"
        "fmull  40(%1)\n\t"
        "fldl   16(%1)\n\t"
        "fmull  56(%1)\n\t"
        "fldl   32(%1)\n\t" /* angle */
        "fsin\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "fstpl  %0\n\t"
        : "=m"(out)
        : "r"(a)
        : "st", "st(1)", "st(2)", "st(3)");
    return out;
}

int main(void) {
    const double a[8] = {1.25, -3.5, 2.0, 0.75, 0.5, -1.125, 4.0, -2.25};

    {
        const double p0 = a[0] * a[3];
        const double p1 = a[1] * a[5];
        const double p2 = a[2] * a[7];
        const double s = sin(a[4]);
        const double expected = p0 + (p1 + (p2 + s));
        check_ulp("products_then_fsin", products_then_fsin(a), expected);
    }

    if (failures == 0) {
        printf("ALL PASS  (0 failures)\n");
    } else {
        printf("SOME FAILURES  (%d failures)\n", failures);
    }
    return failures == 0 ? 0 : 1;
}
