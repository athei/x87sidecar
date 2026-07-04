/*
 * test_ir_remat.c — Numerics for the remat/sink pressure relief.
 *
 * Shape: values loaded early whose only use comes at the very end of a
 * long run (weights applied after a product chain).  Under a clamped FPR
 * pool (run_tests.sh pressure phase) the remat pass sinks those loads to
 * their single late use instead of splitting; results must be identical
 * on every path.  Shuffled offsets keep pass_fma_reduce out.
 *
 * Build: clang -arch x86_64 -O0 -o test_ir_remat test_ir_remat.c
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;
static uint64_t as_u64(double d) {
    uint64_t u;
    memcpy(&u, &d, 8);
    return u;
}

static void check(const char* name, double got, double expected) {
    if (as_u64(got) != as_u64(expected)) {
        printf("FAIL  %-55s  got=%.17g  expected=%.17g\n", name, got, expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

/* w loaded first, single use at the last fmulp — the remat/sink shape. */
static double early_weight_late_use(const double* a, const double* w) {
    double out;
    __asm__ volatile(
        "fldl   (%2)\n\t" /* w[0]: only use is the final fmulp */
        "fldl   (%1)\n\t"
        "fmull  24(%1)\n\t"
        "fldl   8(%1)\n\t"
        "fmull  40(%1)\n\t"
        "fldl   16(%1)\n\t"
        "fmull  56(%1)\n\t"
        "fldl   32(%1)\n\t"
        "fmull  48(%1)\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "fmulp\n\t"
        "fstpl  %0\n\t"
        : "=m"(out)
        : "r"(a), "r"(w)
        : "st", "st(1)", "st(2)", "st(3)", "st(4)");
    return out;
}

/* Two early single-late-use values (w0 multiplied, w1 added at the end). */
static double two_early_values(const double* a, const double* w) {
    double out;
    __asm__ volatile(
        "fldl   8(%2)\n\t" /* w[1]: only use is the final faddp */
        "fldl   (%2)\n\t"  /* w[0]: only use is the fmulp before it */
        "fldl   (%1)\n\t"
        "fmull  16(%1)\n\t"
        "fldl   8(%1)\n\t"
        "fmull  32(%1)\n\t"
        "fldl   24(%1)\n\t"
        "fmull  48(%1)\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "fmulp\n\t"
        "faddp\n\t"
        "fstpl  %0\n\t"
        : "=m"(out)
        : "r"(a), "r"(w)
        : "st", "st(1)", "st(2)", "st(3)", "st(4)");
    return out;
}

int main(void) {
    const double a[8] = {1.25, -3.5, 2.0, 0.75, 5.5, -1.125, 4.0, -2.25};
    const double w[2] = {100.5, -7.25};

    {
        const double p0 = a[0] * a[3];
        const double p1 = a[1] * a[5];
        const double p2 = a[2] * a[7];
        const double p3 = a[4] * a[6];
        const double sum = p0 + (p1 + (p2 + p3));
        check("early_weight_late_use", early_weight_late_use(a, w), sum * w[0]);
    }
    {
        const double p0 = a[0] * a[2];
        const double p1 = a[1] * a[4];
        const double p2 = a[3] * a[6];
        const double sum = p0 + (p1 + p2);
        check("two_early_values", two_early_values(a, w), sum * w[0] + w[1]);
    }

    if (failures == 0) {
        printf("ALL PASS  (0 failures)\n");
    } else {
        printf("SOME FAILURES  (%d failures)\n", failures);
    }
    return failures == 0 ? 0 : 1;
}
