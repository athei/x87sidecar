/*
 * bench_fusion_fld_arith_arithp.c -- Benchmark for the fld_arith_arithp fusion.
 *
 * Pattern: FLD src / ARITH ST(0),ST(1) / ARITHp ST(1)
 *
 * The FMA upgrade folds FLD + FMUL + FADDP/FSUBP into a single ARM64 FMA
 * instruction.  This benchmark measures the FMA-eligible paths and includes
 * a non-FMA baseline for comparison.
 *
 * Build: clang -arch x86_64 -O2 -o bench_fusion_fld_arith_arithp \
 *        bench_fusion_fld_arith_arithp.c
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 1000000
#define RUNS 5

/*
 * Scenario 1: FLD m64 + FMUL ST(0),ST(1) + FADDP — primary FMA target.
 *
 * ST(0)=4.  FLD [3.0] -> FMUL -> FADDP.
 * Result = 4 + 3*4 = 16.
 */
static bench_ns_t bench_fma_reg_faddp(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 4.0, src = 3.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl  %1\n\t"
            "fldl  %2\n\t"
            "fmul  %%st(1), %%st\n\t"
            "faddp\n\t"
            "fstpl %0\n"
            : "=m"(r)
            : "m"(a), "m"(src));
    return bench_now_ns() - start;
}

/*
 * Scenario 2: FLD m64 + FMUL ST(0),ST(1) + FSUBP — FMA subtract variant.
 *
 * GAS fsubp = Intel FSUBRP: ST(1) = ST(0) - ST(1).
 * ST(0)=4.  FLD [3.0] -> FMUL -> FSUBP.
 * Result = 3*4 - 4 = 8.
 */
static bench_ns_t bench_fma_reg_fsubp(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 4.0, src = 3.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl  %1\n\t"
            "fldl  %2\n\t"
            "fmul  %%st(1), %%st\n\t"
            "fsubp\n\t"
            "fstpl %0\n"
            : "=m"(r)
            : "m"(a), "m"(src));
    return bench_now_ns() - start;
}

/*
 * Scenario 3: FLD m64 + FMUL [mem64] + FADDP — memory multiply FMA path.
 *
 * ST(0)=5.  FLD [2.0] -> FMULL [4.0] -> FADDP.
 * Result = 5 + 2*4 = 13.
 */
static bench_ns_t bench_fma_mem_faddp(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 5.0, src = 2.0, mul_val = 4.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl  %1\n\t"
            "fldl  %2\n\t"
            "fmull %3\n\t"
            "faddp\n\t"
            "fstpl %0\n"
            : "=m"(r)
            : "m"(a), "m"(src), "m"(mul_val));
    return bench_now_ns() - start;
}

/*
 * Scenario 4: FLD m64 + FADD + FADDP — non-FMA baseline (arith1=ADD).
 *
 * ST(0)=5.  FLD [2.0] -> FADD ST(0),ST(1) -> FADDP.
 * Result = 5 + (2+5) = 12.
 */
static bench_ns_t bench_nonfma_fadd_faddp(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 5.0, src = 2.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl  %1\n\t"
            "fldl  %2\n\t"
            "fadd  %%st(1), %%st\n\t"
            "faddp\n\t"
            "fstpl %0\n"
            : "=m"(r)
            : "m"(a), "m"(src));
    return bench_now_ns() - start;
}

/*
 * Scenario 5: 3-component dot product using FLD+FMUL+FADDP chain.
 *
 * dot = a0*b0 + a1*b1 + a2*b2
 * First term: FLD a0 + FMULL b0 (handled by fld_arithp).
 * Terms 2-3: FLD ai + FMUL [bi] + FADDP (fld_arith_arithp with FMA).
 *
 * Result = 1*4 + 2*5 + 3*6 = 32.
 */
static bench_ns_t bench_dot3_fma(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a0 = 1.0, a1 = 2.0, a2 = 3.0;
    volatile double b0 = 4.0, b1 = 5.0, b2 = 6.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            /* Term 0: a0*b0 */
            "fldl  %1\n\t"
            "fmull %4\n\t"
            /* Term 1: + a1*b1 (fld_arith_arithp FMA) */
            "fldl  %2\n\t"
            "fmull %5\n\t"
            "faddp\n\t"
            /* Term 2: + a2*b2 (fld_arith_arithp FMA) */
            "fldl  %3\n\t"
            "fmull %6\n\t"
            "faddp\n\t"
            "fstpl %0\n"
            : "=m"(r)
            : "m"(a0), "m"(a1), "m"(a2), "m"(b0), "m"(b1), "m"(b2));
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"fma_reg_faddp", bench_fma_reg_faddp}, {"fma_reg_fsubp", bench_fma_reg_fsubp},
        {"fma_mem_faddp", bench_fma_mem_faddp}, {"nonfma_fadd_faddp", bench_nonfma_fadd_faddp},
        {"dot3_fma", bench_dot3_fma},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        bench_ns_t sum = 0;
        for (int r = 0; r < RUNS; r++)
            sum += benches[i].fn();
        printf("BENCH %s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
