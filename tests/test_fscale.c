/*
 * test_fscale.c — Tests for FSCALE (D9 FD).
 *
 * Spec: ST(0) ← ST(0) * 2^trunc(ST(1)). ST(1) is NOT popped.
 *
 * Truncation of ST(1) is integer truncation (toward zero), not rounding.
 *
 * Edge cases (Intel SDM):
 *   ST(0) NaN          → result = NaN
 *   ST(1) NaN          → result = NaN
 *   ST(0) = 0, ST(1) = ±Inf  → result = NaN  (0·∞ invalid)
 *   ST(0) = ±Inf, ST(1) = ∓Inf → result = NaN
 *   ST(0) finite, ST(1) = +Inf → ±Inf with sign of ST(0)
 *   ST(0) finite, ST(1) = -Inf → ±0 with sign of ST(0)
 *   Overflow (k too large) → ±Inf with sign of ST(0)
 *   Underflow (k too negative) → ±0 with sign of ST(0)
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static double do_fscale(double a, double b) {
    /* fld b; fld a;  fscale  → ST(0) = a * 2^trunc(b), ST(1) = b unchanged.
     * fstpl pops ST(0); fstpl pops ST(1)=b. */
    double r;
    double drained;
    __asm__ volatile(
        "fldl  %2\n\t" /* push b → ST(0)=b */
        "fldl  %3\n\t" /* push a → ST(0)=a, ST(1)=b */
        "fscale\n\t"
        "fstpl %0\n\t" /* pop result */
        "fstpl %1\n\t" /* pop b */
        : "=m"(r), "=m"(drained)
        : "m"(b), "m"(a)
        : "st");
    (void)drained;
    return r;
}

static void check(const char* name, double a, double b, double expected) {
    double got = do_fscale(a, b);
    uint64_t g, e;
    memcpy(&g, &got, 8);
    memcpy(&e, &expected, 8);
    /* For NaN we accept any NaN bit pattern (don't compare bit-exact). */
    int both_nan =
        ((g & 0x7FF0000000000000ULL) == 0x7FF0000000000000ULL &&
         (g & 0x000FFFFFFFFFFFFFULL) != 0) &&
        ((e & 0x7FF0000000000000ULL) == 0x7FF0000000000000ULL && (e & 0x000FFFFFFFFFFFFFULL) != 0);
    int ok = (g == e) || both_nan;
    if (ok) {
        printf("PASS  %s  fscale(%.17g, %.17g) = %.17g\n", name, a, b, got);
    } else {
        printf("FAIL  %s  fscale(%.17g, %.17g)\n", name, a, b);
        printf("      got=%.17g (bits=%016llx)\n", got, (unsigned long long)g);
        printf("      exp=%.17g (bits=%016llx)\n", expected, (unsigned long long)e);
        failures++;
    }
}

int main(void) {
    /* Normal cases. */
    check("fscale(2.0, 3.0)", 2.0, 3.0, 16.0);
    check("fscale(1.0, -2.0)", 1.0, -2.0, 0.25);
    check("fscale(3.0, 0.0)", 3.0, 0.0, 3.0);
    check("fscale(1.5, 1.0)", 1.5, 1.0, 3.0);
    check("fscale(-2.0, 4.0)", -2.0, 4.0, -32.0);
    check("fscale(2.0, 1.7)", 2.0, 1.7, 4.0);       /* trunc(1.7)=1 */
    check("fscale(2.0, -1.5)", 2.0, -1.5, 1.0);     /* trunc(-1.5)=-1 */
    check("fscale(2.0, -1.999)", 2.0, -1.999, 1.0); /* trunc(-1.999)=-1 */

    /* ST(0) zero — preserve sign. */
    {
        double neg_zero;
        uint64_t bits = 0x8000000000000000ULL;
        memcpy(&neg_zero, &bits, 8);
        check("fscale(+0, 5)", 0.0, 5.0, 0.0);
        check("fscale(-0, 5)", neg_zero, 5.0, neg_zero);
    }

    /* ST(0) ±Inf. */
    {
        double pos_inf = 1.0 / 0.0;
        double neg_inf = -1.0 / 0.0;
        check("fscale(+Inf, 1)", pos_inf, 1.0, pos_inf);
        check("fscale(-Inf, 1)", neg_inf, 1.0, neg_inf);
        /* Inf * 2^-Inf = NaN. */
        double nan_v = 0.0 / 0.0;
        check("fscale(+Inf, -Inf)", pos_inf, neg_inf, nan_v);
    }

    /* ST(1) = +Inf → big k → +Inf with sign of ST(0). */
    {
        double pos_inf = 1.0 / 0.0;
        double neg_inf = -1.0 / 0.0;
        check("fscale(2.0, +Inf)", 2.0, pos_inf, pos_inf);
        check("fscale(-3.0, +Inf)", -3.0, pos_inf, neg_inf);
    }

    /* ST(1) = -Inf → tiny k → 0 with sign of ST(0). */
    {
        double neg_inf = -1.0 / 0.0;
        double neg_zero;
        uint64_t bits = 0x8000000000000000ULL;
        memcpy(&neg_zero, &bits, 8);
        check("fscale(2.0, -Inf)", 2.0, neg_inf, 0.0);
        check("fscale(-3.0, -Inf)", -3.0, neg_inf, neg_zero);
    }

    /* 0 * 2^Inf = NaN (invalid). */
    {
        double pos_inf = 1.0 / 0.0;
        double nan_v = 0.0 / 0.0;
        check("fscale(0, +Inf)", 0.0, pos_inf, nan_v);
    }

    /* NaN propagation. */
    {
        double qnan;
        uint64_t bits = 0x7FF8000000000001ULL;
        memcpy(&qnan, &bits, 8);
        double nan_v = 0.0 / 0.0;
        check("fscale(NaN, 1.0)", qnan, 1.0, nan_v);
        check("fscale(2.0, NaN)", 2.0, qnan, nan_v);
    }

    /* Out-of-range k → overflow / underflow. */
    {
        double pos_inf = 1.0 / 0.0;
        check("fscale(2.0, 2000.0)", 2.0, 2000.0, pos_inf); /* overflow → +Inf */
        check("fscale(2.0, -2000.0)", 2.0, -2000.0, 0.0);   /* underflow → 0 */
    }

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
