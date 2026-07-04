/*
 * test_ir_split.c — Numerics for runs that exceed the FPR pressure gate.
 *
 * These runs hold many simultaneously-live values (a full 8-deep stack of
 * products consumed by a reduction tree), the shape that historically made
 * compile_run refuse on FprPressure.  With pressure splitting the run
 * lowers as several IR sub-runs; the values must be identical either way.
 * Run under X87_FPR_POOL_LIMIT (run_tests.sh pressure phase) the splits
 * fire deterministically; in the default phases the same code validates
 * the unsplit path.
 *
 * Build: clang -arch x86_64 -O0 -o test_ir_split test_ir_split.c
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

/* 8 products live at once, then a full reduction tree.  Non-constant
 * strides (shuffled offsets) so pass_fma_reduce cannot absorb the chain —
 * the pressure must be carried by live FPRs. */
static double deep_product_tree(const double* a, const double* b) {
    double out;
    __asm__ volatile(
        "fldl   (%1)\n\t"
        "fmull  56(%2)\n\t"
        "fldl   8(%1)\n\t"
        "fmull  40(%2)\n\t"
        "fldl   16(%1)\n\t"
        "fmull  24(%2)\n\t"
        "fldl   24(%1)\n\t"
        "fmull  8(%2)\n\t"
        "fldl   32(%1)\n\t"
        "fmull  48(%2)\n\t"
        "fldl   40(%1)\n\t"
        "fmull  32(%2)\n\t"
        "fldl   48(%1)\n\t"
        "fmull  16(%2)\n\t"
        "fldl   56(%1)\n\t"
        "fmull  (%2)\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "fstpl  %0\n\t"
        : "=m"(out)
        : "r"(a), "r"(b)
        : "st", "st(1)", "st(2)", "st(3)", "st(4)", "st(5)", "st(6)", "st(7)");
    return out;
}

/* Same pressure shape with mixed sub/add so the tree isn't a pure sum. */
static double deep_mixed_tree(const double* a, const double* b) {
    double out;
    __asm__ volatile(
        "fldl   (%1)\n\t"
        "fmull  8(%2)\n\t"
        "fldl   8(%1)\n\t"
        "fmull  (%2)\n\t"
        "fldl   16(%1)\n\t"
        "fmull  40(%2)\n\t"
        "fldl   24(%1)\n\t"
        "fmull  32(%2)\n\t"
        "fldl   32(%1)\n\t"
        "fmull  56(%2)\n\t"
        "fldl   40(%1)\n\t"
        "fmull  48(%2)\n\t"
        "fsubp\n\t"
        "faddp\n\t"
        "fsubp\n\t"
        "faddp\n\t"
        "fsubp\n\t"
        "fstpl  %0\n\t"
        : "=m"(out)
        : "r"(a), "r"(b)
        : "st", "st(1)", "st(2)", "st(3)", "st(4)", "st(5)");
    return out;
}

int main(void) {
    const double a[8] = {1.5, -2.25, 3.0, 0.5, -4.75, 6.125, -0.375, 8.0};
    const double b[8] = {2.0, 0.25, -1.5, 3.75, 5.0, -0.125, 7.5, -2.5};

    /* Reference values computed in plain C double math — the IR path is
     * f64 throughout, so results must match bit-exactly, split or not.
     * The faddp tree reduces top-down: with products p0..p7 pushed in
     * order (p7 on top), faddp folds p7 into p6, then p6' into p5, ... */
    {
        double p[8];
        p[0] = a[0] * b[7];
        p[1] = a[1] * b[5];
        p[2] = a[2] * b[3];
        p[3] = a[3] * b[1];
        p[4] = a[4] * b[6];
        p[5] = a[5] * b[4];
        p[6] = a[6] * b[2];
        p[7] = a[7] * b[0];
        double acc = p[7];
        for (int i = 6; i >= 0; i--) {
            acc = p[i] + acc;
        }
        check("deep_product_tree (8 live products)", deep_product_tree(a, b), acc);
    }
    {
        double p[6];
        p[0] = a[0] * b[1];
        p[1] = a[1] * b[0];
        p[2] = a[2] * b[5];
        p[3] = a[3] * b[4];
        p[4] = a[4] * b[7];
        p[5] = a[5] * b[6];
        /* GAS AT&T `fsubp` assembles to Intel FSUBRP: st(1) = st(0) - st(1),
         * pop (the notorious AT&T swap; see test_arith.c header). */
        double acc = p[5] - p[4];
        acc = p[3] + acc;
        acc = acc - p[2];
        acc = p[1] + acc;
        acc = acc - p[0];
        check("deep_mixed_tree (sub/add tree)", deep_mixed_tree(a, b), acc);
    }

    if (failures == 0) {
        printf("ALL PASS  (0 failures)\n");
    } else {
        printf("SOME FAILURES  (%d failures)\n", failures);
    }
    return failures == 0 ? 0 : 1;
}
