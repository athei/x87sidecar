/*
 * test_fma_reduce_strided.c — bit-exact validation of the *strided* and
 * *f64* FMA-reduction lowering added in Phase 1 (X87IROptimize.cpp::
 * pass_fma_reduce parametric stride + X87IRLower.cpp::lower_fma_reduce
 * strided lane loads / LDR Q).
 *
 * test_fma_reduce.c already covers the +4-contiguous f32 fast path.  This
 * file exercises the cases that path does not:
 *
 *   - f32 with stride != 4 (matrix-vector idiom)  → LDR S + LD1 {V.S}[1]
 *   - f32 with independent L/W strides (one contiguous, one strided)
 *   - f64 contiguous (stride 8 = element size)     → LDR Q (.2D, no FCVTL)
 *   - f64 strided (stride 16)                      → LDR D + LD1 {V.D}[1]
 *   - odd-length chains in both the strided-f32 and strided-f64 forms
 *
 * As in test_fma_reduce.c, inputs are small non-negative integers so every
 * product and partial sum is exactly representable in f64; the re-associated
 * lane-pair vector result is therefore bit-identical to the scalar serial
 * oracle.  Run under Phase 2 (vector path, X87_ENABLE_FMA_REDUCE default ON)
 * and Phase 6 (X87_ENABLE_FMA_REDUCE=0, scalar FMADD) — both must agree.
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

static void check_equal(const char* name, double got, double expected) {
    if (as_u64(got) != as_u64(expected)) {
        printf("FAIL  %-44s got=%.17g (0x%016llx) expected=%.17g (0x%016llx)\n", name, got,
               (unsigned long long)as_u64(got), expected, (unsigned long long)as_u64(expected));
        failures++;
    } else {
        printf("PASS  %-44s = %.17g\n", name, got);
    }
}

/* 16-aligned f32 / f64 backing stores.  Element i lives at sdata[i]; the asm
 * picks elements out by byte offset, so a stride-8 f32 chain reads sdata[0],
 * sdata[2], sdata[4], …  (offsets 0, 8, 16, …). */
static float sdata[16] __attribute__((aligned(16)));
static float sweight[16] __attribute__((aligned(16)));
static double ddata[16] __attribute__((aligned(16)));
static double dweight[16] __attribute__((aligned(16)));

/* ── f32, stride 8 (both streams) ─────────────────────────────────────────── */

static void f32_stride8_n4(double init, double* out) {
    __asm__ volatile(
        "fldl %2\n\t"
        "flds 0(%0)\n\t   fmuls 0(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
        "flds 8(%0)\n\t   fmuls 8(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
        "flds 16(%0)\n\t  fmuls 16(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "flds 24(%0)\n\t  fmuls 24(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "fstpl %3\n"
        :
        : "r"(sdata), "r"(sweight), "m"(init), "m"(*out)
        : "memory");
}

static void f32_stride8_n5(double init, double* out) {
    __asm__ volatile(
        "fldl %2\n\t"
        "flds 0(%0)\n\t   fmuls 0(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
        "flds 8(%0)\n\t   fmuls 8(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
        "flds 16(%0)\n\t  fmuls 16(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "flds 24(%0)\n\t  fmuls 24(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "flds 32(%0)\n\t  fmuls 32(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "fstpl %3\n"
        :
        : "r"(sdata), "r"(sweight), "m"(init), "m"(*out)
        : "memory");
}

/* ── f32, L contiguous (stride 4) but W strided (stride 8) ─────────────────── */

static void f32_mixed_n4(double init, double* out) {
    __asm__ volatile(
        "fldl %2\n\t"
        "flds 0(%0)\n\t   fmuls 0(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
        "flds 4(%0)\n\t   fmuls 8(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
        "flds 8(%0)\n\t   fmuls 16(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "flds 12(%0)\n\t  fmuls 24(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "fstpl %3\n"
        :
        : "r"(sdata), "r"(sweight), "m"(init), "m"(*out)
        : "memory");
}

/* ── f64, contiguous (stride 8 = element size) → LDR Q path ────────────────── */

static void f64_contig_n4(double init, double* out) {
    __asm__ volatile(
        "fldl %2\n\t"
        "fldl 0(%0)\n\t   fmull 0(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
        "fldl 8(%0)\n\t   fmull 8(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
        "fldl 16(%0)\n\t  fmull 16(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "fldl 24(%0)\n\t  fmull 24(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "fstpl %3\n"
        :
        : "r"(ddata), "r"(dweight), "m"(init), "m"(*out)
        : "memory");
}

/* ── f64, strided (stride 16) → LDR D + LD1 {V.D}[1] path ──────────────────── */

static void f64_stride16_n4(double init, double* out) {
    __asm__ volatile(
        "fldl %2\n\t"
        "fldl 0(%0)\n\t   fmull 0(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
        "fldl 16(%0)\n\t  fmull 16(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "fldl 32(%0)\n\t  fmull 32(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "fldl 48(%0)\n\t  fmull 48(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "fstpl %3\n"
        :
        : "r"(ddata), "r"(dweight), "m"(init), "m"(*out)
        : "memory");
}

static void f64_stride16_n3(double init, double* out) {
    __asm__ volatile(
        "fldl %2\n\t"
        "fldl 0(%0)\n\t   fmull 0(%1)\n\t   faddp %%st(0), %%st(1)\n\t"
        "fldl 16(%0)\n\t  fmull 16(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "fldl 32(%0)\n\t  fmull 32(%1)\n\t  faddp %%st(0), %%st(1)\n\t"
        "fstpl %3\n"
        :
        : "r"(ddata), "r"(dweight), "m"(init), "m"(*out)
        : "memory");
}

/* Oracles — read the same elements the asm does, accumulate left-to-right in
 * f64.  `lm`/`wm` are the per-stream element-index multipliers (stride / size). */
static double exp_f32(double init, int n, int lm, int wm) {
    double acc = init;
    for (int i = 0; i < n; i++) {
        acc += (double)sdata[lm * i] * (double)sweight[wm * i];
    }
    return acc;
}

static double exp_f64(double init, int n, int sm) {
    double acc = init;
    for (int i = 0; i < n; i++) {
        acc += ddata[sm * i] * dweight[sm * i];
    }
    return acc;
}

int main(void) {
    for (int i = 0; i < 16; i++) {
        sdata[i] = (float)(i + 1);
        sweight[i] = (float)(i + 1);
        ddata[i] = (double)(i + 1);
        dweight[i] = (double)(i + 1);
    }

    double got;

    /* f32 stride-8: even pairs, two FMLA iterations. */
    f32_stride8_n4(100.0, &got);
    check_equal("f32 stride8 n4 init=100", got, exp_f32(100.0, 4, 2, 2));

    /* f32 stride-8 odd length: two pairs + scalar odd tail. */
    f32_stride8_n5(0.0, &got);
    check_equal("f32 stride8 n5 init=0 (odd tail)", got, exp_f32(0.0, 5, 2, 2));

    /* f32 independent strides: L contiguous (×1), W strided (×2). */
    f32_mixed_n4(-3.0, &got);
    check_equal("f32 mixed L4/W8 n4 init=-3", got, exp_f32(-3.0, 4, 1, 2));

    /* f64 contiguous → LDR Q. */
    f64_contig_n4(100.0, &got);
    check_equal("f64 contig n4 init=100 (LDR Q)", got, exp_f64(100.0, 4, 1));

    f64_contig_n4(0.0, &got);
    check_equal("f64 contig n4 init=0 (LDR Q)", got, exp_f64(0.0, 4, 1));

    /* f64 stride-16 → LD1 {V.D}[1]. */
    f64_stride16_n4(50.0, &got);
    check_equal("f64 stride16 n4 init=50", got, exp_f64(50.0, 4, 2));

    /* f64 stride-16 odd length: one pair + scalar f64 odd tail. */
    f64_stride16_n3(0.0, &got);
    check_equal("f64 stride16 n3 init=0 (odd tail)", got, exp_f64(0.0, 3, 2));

    /* Mixed signs, strided f32 — partial sums stay exact. */
    for (int i = 0; i < 16; i++) {
        sdata[i] = (i % 2 == 0) ? (float)(i + 1) : -(float)(i + 1);
    }
    f32_stride8_n4(0.0, &got);
    check_equal("f32 stride8 n4 mixed signs", got, exp_f32(0.0, 4, 2, 2));

    if (failures == 0) {
        printf("\nALL PASS\n");
        return 0;
    }
    printf("\n%d FAIL(s)\n", failures);
    return 1;
}
