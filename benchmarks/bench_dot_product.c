/*
 * bench_dot_product.c — Benchmark for matrix-vector dot-product chains
 * `(fld m32 + fmul m32 + faddp st1, st0) × N` against contiguous f32
 * data and weight streams.
 *
 * Models the dominant hot pattern in the 2026-05-06 WoW capture
 * (rank-1 / rank-2 / rank-10 / rank-12 in /tmp/epoch.prof, ≈20% of
 * total exec-weighted ARM emit).
 *
 * Three configurations exercise the relevant lowering paths:
 *   - X87_DISABLE_HOOK=1                  (stock Rosetta, baseline)
 *   - default                             (scalar FMADD chain via FMA pass)
 *   - X87_ENABLE_FMA_REDUCE=1             (NEON FMLA .2D reduction)
 *
 * Per the plan in ~/.claude/plans/elegant-petting-hammock.md, the
 * NEON path's expected per-pair cost is ~5 ARM (LDR D × 2 + FCVTL × 2
 * + FMLA .2D) covering 2 trios, vs ~10 ARM for two scalar FMADD trios.
 * We expect ≥1.3× speedup on n_8 / n_16 chains (LDR-D path; ILP and
 * memory subsystem absorb some of the ARM-instruction win).
 *
 * Run via the rosettax87 loader (per feedback_runtime_loader.md).
 */
#include <stdint.h>
#include <stdio.h>

#include "bench_timing.h"

#define TIMES 1000000
#define RUNS 5

static float data[16] __attribute__((aligned(16)));
static float weight[16] __attribute__((aligned(16)));
static double init = 1.0;

/* N=4: two paired vector iterations, no odd tail. */
static bench_ns_t bench_dot_n4(void) {
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        double out;
        __asm__ volatile(
            "fldl %2\n\t"
            "flds 0(%0)\n\t   fmuls 0(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
            "flds 4(%0)\n\t   fmuls 4(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
            "flds 8(%0)\n\t   fmuls 8(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
            "flds 12(%0)\n\t  fmuls 12(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
            "fstpl %3\n"
            :
            : "r"(data), "r"(weight), "m"(init), "m"(out)
            : "memory");
    }
    return bench_now_ns() - start;
}

/* N=8: four paired vector iterations, no odd tail (matches WoW rank-2
 * 30-op dot-product subpattern length). */
static bench_ns_t bench_dot_n8(void) {
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        double out;
        __asm__ volatile(
            "fldl %2\n\t"
            "flds 0(%0)\n\t   fmuls 0(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
            "flds 4(%0)\n\t   fmuls 4(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
            "flds 8(%0)\n\t   fmuls 8(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
            "flds 12(%0)\n\t  fmuls 12(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
            "flds 16(%0)\n\t  fmuls 16(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
            "flds 20(%0)\n\t  fmuls 20(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
            "flds 24(%0)\n\t  fmuls 24(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
            "flds 28(%0)\n\t  fmuls 28(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
            "fstpl %3\n"
            :
            : "r"(data), "r"(weight), "m"(init), "m"(out)
            : "memory");
    }
    return bench_now_ns() - start;
}

/* N=3: one paired iter + one odd-trio scalar tail.  Smaller chains
 * exercise the odd-tail FMADD path. */
static bench_ns_t bench_dot_n3(void) {
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        double out;
        __asm__ volatile(
            "fldl %2\n\t"
            "flds 0(%0)\n\t  fmuls 0(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
            "flds 4(%0)\n\t  fmuls 4(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
            "flds 8(%0)\n\t  fmuls 8(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
            "fstpl %3\n"
            :
            : "r"(data), "r"(weight), "m"(init), "m"(out)
            : "memory");
    }
    return bench_now_ns() - start;
}

int main(void) {
    for (int i = 0; i < 16; i++) {
        data[i] = (float)(i + 1) * 0.5f;
        weight[i] = (float)(i + 1) * 0.25f;
    }

    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"dot_product_n3", bench_dot_n3},
        {"dot_product_n4", bench_dot_n4},
        {"dot_product_n8", bench_dot_n8},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        benches[i].fn(); /* warmup: discard, JIT translates on first call */
        bench_ns_t sum = 0;
        for (int r = 0; r < RUNS; r++)
            sum += benches[i].fn();
        printf("BENCH %s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
