/*
 * test_fxtract.c — Tests for FXTRACT (D9 F4).
 *
 * Spec: ST(0) ← unbiased exponent (as f64); push significand (as f64).
 *       After: ST(0) = significand, ST(1) = exponent.
 *
 * Edge cases (Intel SDM §FXTRACT):
 *   ±0:    exp = -∞, sig = ±0 (sign preserved)
 *   ±∞:    exp = +∞ always (regardless of input sign), sig = ±∞
 *   NaN:   both exp and sig are QNaN
 *   normal: sig = sign | 1023<<52 | mantissa, exp = unbiased_exp
 *
 * Denormals (subnormal inputs) are SDM-defined as a normalized result;
 * we treat them as the zero case (exp=-∞, sig=input). No tests exercise
 * denormals here.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void do_fxtract(double in, double *sig, double *exp_out) {
    /* fld in;  fxtract;  fstpl sig (pops new ST(0)=sig);  fstpl exp. */
    __asm__ volatile (
        "fldl  %2\n\t"
        "fxtract\n\t"
        "fstpl %0\n\t"
        "fstpl %1\n\t"
        : "=m"(*sig), "=m"(*exp_out)
        : "m"(in)
        : "st");
}

static void check_pair(const char *name, double in, double exp_sig, double exp_exp) {
    double sig, exp_v;
    do_fxtract(in, &sig, &exp_v);
    uint64_t s_bits, es_bits, e_bits, ee_bits;
    memcpy(&s_bits,  &sig,     8);
    memcpy(&es_bits, &exp_sig, 8);
    memcpy(&e_bits,  &exp_v,   8);
    memcpy(&ee_bits, &exp_exp, 8);
    int ok = (s_bits == es_bits) && (e_bits == ee_bits);
    if (ok) {
        printf("PASS  %s  sig=%.17g exp=%.17g\n", name, sig, exp_v);
    } else {
        printf("FAIL  %s  in=%.17g\n", name, in);
        printf("      sig got=%.17g (bits=%016llx)  expected=%.17g (bits=%016llx)\n",
               sig, (unsigned long long)s_bits, exp_sig, (unsigned long long)es_bits);
        printf("      exp got=%.17g (bits=%016llx)  expected=%.17g (bits=%016llx)\n",
               exp_v, (unsigned long long)e_bits, exp_exp, (unsigned long long)ee_bits);
        failures++;
    }
}

int main(void) {
    /* Normal cases — sig in [1.0, 2.0), exp = unbiased exponent. */
    check_pair("fxtract(1.0)",     1.0,     1.0,    0.0);
    check_pair("fxtract(2.0)",     2.0,     1.0,    1.0);
    check_pair("fxtract(0.5)",     0.5,     1.0,   -1.0);
    check_pair("fxtract(6.0)",     6.0,     1.5,    2.0);
    check_pair("fxtract(-6.0)",   -6.0,    -1.5,    2.0);
    check_pair("fxtract(1.75)",    1.75,    1.75,   0.0);
    check_pair("fxtract(1024.0)",  1024.0,  1.0,   10.0);
    check_pair("fxtract(0.0625)",  0.0625,  1.0,   -4.0);

    /* ±0 → exp=-∞, sig preserves sign. */
    {
        double neg_inf = -1.0 / 0.0;
        check_pair("fxtract(+0)", 0.0, 0.0, neg_inf);
        /* For -0, expected sig is -0.  Construct it explicitly. */
        double neg_zero;
        uint64_t bits = 0x8000000000000000ULL;
        memcpy(&neg_zero, &bits, 8);
        check_pair("fxtract(-0)", neg_zero, neg_zero, neg_inf);
    }

    /* ±Inf → exp=+Inf always; sig preserves sign of Inf. */
    {
        double pos_inf =  1.0 / 0.0;
        double neg_inf = -1.0 / 0.0;
        check_pair("fxtract(+Inf)", pos_inf, pos_inf, pos_inf);
        check_pair("fxtract(-Inf)", neg_inf, neg_inf, pos_inf);
    }

    /* NaN → both QNaN. We use a specific QNaN payload and require bit-exact
     * preservation (input passed through both outputs). */
    {
        double qnan;
        uint64_t bits = 0x7FF8000000000001ULL;  /* QNaN with payload */
        memcpy(&qnan, &bits, 8);
        check_pair("fxtract(QNaN)", qnan, qnan, qnan);
    }

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
