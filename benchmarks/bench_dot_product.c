/*
 * bench_dot_product.c: benchmark for matrix-vector dot-product chains
 * `(fld m32 + fmul m32 + faddp st1, st0) x N` against contiguous f32
 * data and weight streams.
 *
 * Models the dominant hot pattern of a WoW 1.12 profile, where these
 * chains were about 20% of the exec-weighted ARM emit.
 *
 * Three configurations exercise the relevant lowering paths:
 *   - X87_DISABLE_HOOK=1                  (stock Rosetta, baseline)
 *   - default                             (scalar FMADD chain via FMA pass)
 *   - X87_ENABLE_FMA_REDUCE=1             (NEON FMLA .2D reduction)
 *
 * The NEON path's expected per-pair cost is ~5 ARM (LDR D x 2 + FCVTL x 2
 * + FMLA .2D) covering 2 trios, vs ~10 ARM for two scalar FMADD trios,
 * so n_8 / n_16 chains should gain at least 1.3x on the LDR-D path; ILP
 * and the memory subsystem absorb some of the ARM-instruction win.
 *
 * Run through the x87sidecar loader; bare, the x87 is not translated by us.
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
