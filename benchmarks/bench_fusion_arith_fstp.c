/*
 * bench_fusion_arith_fstp.c -- Benchmark for arith_fstp fusion (OPT-F14).
 * Pattern: non-popping ARITH + FSTP mem.
 *
 * IMPORTANT: The 3-instruction fld_arith_fstp fusion fires first and eats
 * FLD+ARITH+FSTP triplets.  To benchmark arith_fstp in isolation, we must
 * use patterns where the arithmetic is NOT preceded by an FLD that forms
 * the 3-instr fusion:
 *
 *   1. Values already on the stack from prior operations
 *   2. ARITH with deeper register sources (ST(2)+ rejected by fld_arith_fstp)
 *   3. ARITH following non-FLD instructions (e.g. another ARITH)
 *
 * NOTE: GAS AT&T syntax reverses Intel operand order for non-commutative ops.
 */
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "bench_timing.h"

#define TIMES 1000000
#define RUNS  5

/*
 * Scenario 1: FMUL ST(0),ST(2) + FSTP -- deeper register source.
 *
 * fld_arith_fstp requires src=ST(1) for register form, so ST(2) falls through
 * to arith_fstp.
 *
 * x87: fld [c] | fld [b] | fld [a] | fmul ST(0),ST(2) | fstp [out]
 *                                     ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *                                     arith_fstp fires on these two
 */
static bench_ns_t bench_deep_reg_mul(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 2.0, b = 3.0, c = 5.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile (
            "fldl  %1\n\t"            /* ST(0)=c=5 */
            "fldl  %2\n\t"            /* ST(0)=b=3, ST(1)=5 */
            "fldl  %3\n\t"            /* ST(0)=a=2, ST(1)=3, ST(2)=5 */
            "fmul  %%st(2), %%st\n\t" /* ST(0)=2*5=10 -- arith_fstp target */
            "fstpl %0\n\t"            /* r=10, pop */
            "fstp  %%st(0)\n\t"       /* pop b */
            "fstp  %%st(0)\n"         /* pop c */
            : "=m"(r) : "m"(c), "m"(b), "m"(a));
    return bench_now_ns() - start;
}

/*
 * Scenario 2: FADD ST(0),ST(2) + FSTP -- deeper register add.
 */
static bench_ns_t bench_deep_reg_add(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 10.0, b = 20.0, c = 30.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile (
            "fldl  %1\n\t"
            "fldl  %2\n\t"
            "fldl  %3\n\t"
            "fadd  %%st(2), %%st\n\t"
            "fstpl %0\n\t"
            "fstp  %%st(0)\n\t"
            "fstp  %%st(0)\n"
            : "=m"(r) : "m"(c), "m"(b), "m"(a));
    return bench_now_ns() - start;
}

/*
 * Scenario 3: Two consecutive arith+fstp from stacked values.
 *
 * x87: fld [s] | fld [x] | fld [y]
 *      fmul ST(0),ST(2) | fstp [ry]   -- arith_fstp #1
 *      fmul ST(0),ST(1) | fstp [rx]   -- arith_fstp #2 (src=ST(1) but no FLD before)
 *      fstp ST(0)
 *
 * After first arith+fstp: ST(0)=x, ST(1)=s
 * The second fmul is NOT preceded by FLD so fld_arith_fstp can't fire.
 */
static bench_ns_t bench_double_scale(void) {
    bench_ns_t start = bench_now_ns();
    volatile double x = 3.0, y = 4.0, s = 2.0;
    volatile double rx, ry;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile (
            "fldl  %2\n\t"            /* ST(0)=s */
            "fldl  %3\n\t"            /* ST(0)=x, ST(1)=s */
            "fldl  %4\n\t"            /* ST(0)=y, ST(1)=x, ST(2)=s */
            "fmul  %%st(2), %%st\n\t" /* ST(0)=y*s, ST(1)=x, ST(2)=s -- arith_fstp */
            "fstpl %0\n\t"            /* ry=y*s, pop -> ST(0)=x, ST(1)=s */
            "fmul  %%st(1), %%st\n\t" /* ST(0)=x*s, ST(1)=s -- arith_fstp */
            "fstpl %1\n\t"            /* rx=x*s, pop -> ST(0)=s */
            "fstp  %%st(0)\n"
            : "=m"(ry), "=m"(rx) : "m"(s), "m"(x), "m"(y));
    return bench_now_ns() - start;
}

/*
 * Scenario 4: FDIV ST(0),ST(2) + FSTP m32 -- deep reg, f32 truncation.
 */
static bench_ns_t bench_deep_reg_div_f32(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 15.0, b = 99.0, c = 5.0;
    volatile float r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile (
            "fldl  %1\n\t"
            "fldl  %2\n\t"
            "fldl  %3\n\t"
            "fdiv  %%st(2), %%st\n\t"
            "fstps %0\n\t"
            "fstp  %%st(0)\n\t"
            "fstp  %%st(0)\n"
            : "=m"(r) : "m"(c), "m"(b), "m"(a));
    return bench_now_ns() - start;
}

/*
 * Scenario 5: 3-component dot product residual.
 *
 * Pattern: fld|fmul (fused as fld_arithp), fld|fmul|faddp (fused as fld_arith_arithp),
 *          fld|fmul (fused as fld_arithp), faddp|fstp (fused as arithp_fstp).
 *
 * Compare against: break the chain so arith_fstp fires instead.
 * Here: values pre-loaded, then multiply-accumulate with arith+fstp at the end.
 *
 * fld [a] | fmul [x]     -- fld_arithp fuses
 * fld [b] | fmul [y]     -- fld_arithp fuses
 * faddp                   -- standalone (no fstp after)
 * fld [c] | fmul ST(0),ST(2) | fstp [out]
 *                ^^^^^^^^^^^^^^^^^^^^^^^^^ arith_fstp fires (ST(2) rejects fld_arith_fstp)
 *
 * Wait, that still has FLD before. Let me use a pattern where values are
 * pre-loaded and the final arith+fstp doesn't have a preceding FLD.
 */
static bench_ns_t bench_accum_store(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 1.0, b = 2.0, c = 3.0, d = 4.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile (
            "fldl  %1\n\t"            /* ST(0)=a */
            "fldl  %2\n\t"            /* ST(0)=b, ST(1)=a */
            "faddp\n\t"               /* ST(0)=a+b=3 */
            "fldl  %3\n\t"            /* ST(0)=c, ST(1)=3 */
            "faddp\n\t"               /* ST(0)=a+b+c=6 */
            "fmull %4\n\t"            /* ST(0)=6*4=24 -- arith_fstp target */
            "fstpl %0\n"              /* r=24 */
            : "=m"(r) : "m"(a), "m"(b), "m"(c), "m"(d));
    return bench_now_ns() - start;
}

int main(void) {
    struct { const char *name; bench_ns_t (*fn)(void); } benches[] = {
        {"deep_mul",     bench_deep_reg_mul},
        {"deep_add",     bench_deep_reg_add},
        {"double_scale", bench_double_scale},
        {"deep_div_f32", bench_deep_reg_div_f32},
        {"accum_store",  bench_accum_store},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        bench_ns_t sum = 0;
        for (int r = 0; r < RUNS; r++) sum += benches[i].fn();
        printf("BENCH %s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
