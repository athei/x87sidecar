/*
 * test_x87_loop.c — transcendentals inside a hot loop with live guest
 * state across the op.
 *
 * Regression test for the guest-register clobber class of bug: the loop
 * accumulator lives in an XMM register (v0-v15 on the Rosetta side) and
 * the loop counter in a guest GPR, both live ACROSS each x87 op.  The
 * transcendental emitters used to compute in d0/d1/d2 — guest xmm0/1/2 —
 * so `acc += sin(x)` collapsed to `acc = sin(x) + sin(x)` regardless of
 * the iteration count (the original TurtleWoW-crash repro shape).
 *
 * Straight-line single-op tests can never catch this: the clobbered
 * registers hold nothing live there.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Accumulated sums compare against a libm reference accumulated in the
   same order.  Per-element polynomial-vs-libm error (<= ~4 ULP) shifts
   the whole accumulation trajectory, so sums can land a few thousand
   ULP from the reference; 4096 covers that while still catching the
   clobber failure mode, which is off by orders of magnitude (2*sin vs
   N*sin). */
#define SUM_MAX_ULP 4096ULL

#define N 100000

static int failures = 0;

/* Reference sum accumulated sequentially, mirroring the loop's rounding
   trajectory (a single N*value multiply diverges from 100000 chained
   adds by thousands of ULP — even on native Rosetta). */
static double ref_sum(double per_element) {
    double ref = 0;
    for (int i = 0; i < N; i++) {
        ref += per_element;
    }
    return ref;
}


static void check_sum(const char* name, double got, double expected) {
    uint64_t g, e;
    memcpy(&g, &got, sizeof(g));
    memcpy(&e, &expected, sizeof(e));

    uint64_t g_abs = g & 0x7fffffffffffffffULL;
    uint64_t e_abs = e & 0x7fffffffffffffffULL;
    uint64_t ulp_delta;
    if ((g >> 63) == (e >> 63)) {
        ulp_delta = (g_abs > e_abs) ? (g_abs - e_abs) : (e_abs - g_abs);
    } else {
        ulp_delta = g_abs + e_abs;
    }

    if (g == e || ulp_delta <= SUM_MAX_ULP) {
        printf("PASS  %-28s  got=%.17g [ulp=%llu]\n", name, got, (unsigned long long)ulp_delta);
        return;
    }
    printf("FAIL  %-28s  got=%.17g  expected=%.17g  ulp=%llu\n", name, got, expected,
           (unsigned long long)ulp_delta);
    failures++;
}

int main(void) {
    volatile double x = 1.3;
    volatile double y = 1.5;
    volatile double half = 0.5;
    double acc, r, q;
    int i;

    /* fsin loop — the original repro: expect N*sin(1.3), the broken
       emitter produced 2*sin(1.3). */
    acc = 0;
    for (i = 0; i < N; i++) {
        __asm__ volatile("fldl %[x]\n\tfsin\n\tfstpl %[r]" : [r] "=m"(r) : [x] "m"(x) : "st");
        acc += r;
    }
    check_sum("loop-fsin", acc, ref_sum(sin(1.3)));

    /* fsin loop with varying input: a second live XMM value (the input)
       and a data dependency on the counter. */
    acc = 0;
    for (i = 0; i < N; i++) {
        double xi = 0.5 + (double)i * 1e-6;
        __asm__ volatile("fldl %[x]\n\tfsin\n\tfstpl %[r]" : [r] "=m"(r) : [x] "m"(xi) : "st");
        acc += r;
    }
    {
        double ref = 0;
        for (i = 0; i < N; i++) {
            ref += sin(0.5 + (double)i * 1e-6);
        }
        check_sum("loop-fsin-varying", acc, ref);
    }

    /* fcos */
    acc = 0;
    for (i = 0; i < N; i++) {
        __asm__ volatile("fldl %[x]\n\tfcos\n\tfstpl %[r]" : [r] "=m"(r) : [x] "m"(x) : "st");
        acc += r;
    }
    check_sum("loop-fcos", acc, ref_sum(cos(1.3)));

    /* fsincos — two results, two live accumulators (xmm0 AND xmm1 class) */
    acc = 0;
    {
        double acc2 = 0;
        for (i = 0; i < N; i++) {
            __asm__ volatile("fldl %[x]\n\tfsincos\n\tfstpl %[q]\n\tfstpl %[r]"
                             : [r] "=m"(r), [q] "=m"(q)
                             : [x] "m"(x)
                             : "st", "st(1)");
            acc += r;   /* sin */
            acc2 += q;  /* cos */
        }
        check_sum("loop-fsincos-sin", acc, ref_sum(sin(1.3)));
        check_sum("loop-fsincos-cos", acc2, ref_sum(cos(1.3)));
    }

    /* fptan — result + pushed 1.0 */
    acc = 0;
    for (i = 0; i < N; i++) {
        __asm__ volatile("fldl %[x]\n\tfptan\n\tfstpl %[q]\n\tfstpl %[r]"
                         : [r] "=m"(r), [q] "=m"(q)
                         : [x] "m"(x)
                         : "st", "st(1)");
        acc += r + q;
    }
    check_sum("loop-fptan", acc, ref_sum(tan(1.3) + 1.0));

    /* f2xm1 (spec input |x| <= 1) */
    acc = 0;
    for (i = 0; i < N; i++) {
        __asm__ volatile("fldl %[x]\n\tf2xm1\n\tfstpl %[r]" : [r] "=m"(r) : [x] "m"(half) : "st");
        acc += r;
    }
    check_sum("loop-f2xm1", acc, ref_sum(exp2(0.5) - 1.0));

    /* fpatan */
    acc = 0;
    for (i = 0; i < N; i++) {
        __asm__ volatile("fldl %[y]\n\tfldl %[x]\n\tfpatan\n\tfstpl %[r]"
                         : [r] "=m"(r)
                         : [y] "m"(y), [x] "m"(x)
                         : "st", "st(1)");
        acc += r;
    }
    check_sum("loop-fpatan", acc, ref_sum(atan2(1.5, 1.3)));

    /* fyl2x */
    acc = 0;
    for (i = 0; i < N; i++) {
        __asm__ volatile("fldl %[y]\n\tfldl %[x]\n\tfyl2x\n\tfstpl %[r]"
                         : [r] "=m"(r)
                         : [y] "m"(y), [x] "m"(x)
                         : "st", "st(1)");
        acc += r;
    }
    check_sum("loop-fyl2x", acc, ref_sum(1.5 * log2(1.3)));

    /* fyl2xp1 (spec range for ST(0): small) */
    acc = 0;
    for (i = 0; i < N; i++) {
        __asm__ volatile("fldl %[y]\n\tfldl %[x]\n\tfyl2xp1\n\tfstpl %[r]"
                         : [r] "=m"(r)
                         : [y] "m"(y), [x] "m"(half)
                         : "st", "st(1)");
        acc += r;
    }
    check_sum("loop-fyl2xp1", acc, ref_sum(1.5 * log2(1.5)));

    /* fprem */
    acc = 0;
    for (i = 0; i < N; i++) {
        __asm__ volatile("fldl %[y]\n\tfldl %[x]\n\tfprem\n\tfstpl %[r]\n\tfstpl %[q]"
                         : [r] "=m"(r), [q] "=m"(q)
                         : [y] "m"(half), [x] "m"(x)
                         : "st", "st(1)");
        acc += r;
    }
    check_sum("loop-fprem", acc, ref_sum(fmod(1.3, 0.5)));

    /* fscale */
    acc = 0;
    for (i = 0; i < N; i++) {
        __asm__ volatile("fldl %[y]\n\tfldl %[x]\n\tfscale\n\tfstpl %[r]\n\tfstpl %[q]"
                         : [r] "=m"(r), [q] "=m"(q)
                         : [y] "m"(y), [x] "m"(x)
                         : "st", "st(1)");
        acc += r;
    }
    check_sum("loop-fscale", acc, ref_sum(1.3 * 2.0) /* scalbn(1.3, trunc(1.5)) */);

    if (failures == 0) {
        printf("test_x87_loop: all PASS\n");
        return 0;
    }
    printf("test_x87_loop: %d failure(s)\n", failures);
    return 1;
}
