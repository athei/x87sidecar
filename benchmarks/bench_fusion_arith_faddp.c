/*
 * bench_fusion_arith_faddp.c -- Benchmark for arith_faddp fusion (OPT-F15).
 * Pattern: FMUL + FADDP/FSUBP → FMADD/FMSUB.
 *
 * IMPORTANT: The 3-instruction fld_arith_arithp fusion fires first and eats
 * FLD+FMUL+FADDP triplets.  To benchmark arith_faddp in isolation, we must
 * use patterns where the FMUL is NOT preceded by an FLD that forms the
 * 3-instr fusion:
 *
 *   1. FMUL with memory operand (no FLD involved)
 *   2. FMUL with deeper register sources (ST(2)+ rejected by fld_arith_arithp)
 *   3. FMUL following non-FLD instructions (e.g. another arithmetic op)
 *
 * NOTE: GAS AT&T syntax reverses Intel operand order for non-commutative ops.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 1000000
#define RUNS 5

/*
 * Scenario 1: FMULL [mem64] + FADDP — memory multiply-accumulate.
 *
 * No FLD before FMUL, so fld_arith_arithp cannot fire.
 * Pattern: fld [a] | fld [b] | fmull [c] | faddp
 *          → b*c + a
 */
static bench_ns_t bench_madd_mem(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 2.0, b = 3.0, c = 5.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl  %1\n\t" /* ST(0)=a=2 */
            "fldl  %2\n\t" /* ST(0)=b=3, ST(1)=2 */
            "fmull %3\n\t" /* ST(0)=3*5=15, ST(1)=2 -- arith_faddp target */
            "faddp\n\t"    /* ST(0)=2+15=17 */
            "fstpl %0\n"
            : "=m"(r)
            : "m"(a), "m"(b), "m"(c));
    return bench_now_ns() - start;
}

/*
 * Scenario 2: FMUL ST(0),ST(2) + FADDP — deep register source.
 *
 * fld_arith_arithp requires the middle arith to use ST(0) op ST(1).
 * Using ST(2) bypasses the 3-instr fusion.
 */
static bench_ns_t bench_madd_deep_reg(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 2.0, b = 3.0, c = 5.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl  %1\n\t"            /* ST(0)=c=5 */
            "fldl  %2\n\t"            /* ST(0)=b=3, ST(1)=5 */
            "fldl  %3\n\t"            /* ST(0)=a=2, ST(1)=3, ST(2)=5 */
            "fmul  %%st(2), %%st\n\t" /* ST(0)=2*5=10, ST(1)=3, ST(2)=5 -- arith_faddp */
            "faddp\n\t"               /* ST(0)=3+10=13, ST(1)=5 */
            "fstpl %0\n\t"
            "fstp  %%st(0)\n"
            : "=m"(r)
            : "m"(c), "m"(b), "m"(a));
    return bench_now_ns() - start;
}

/*
 * Scenario 3: 3-component dot product using memory FMUL+FADDP.
 *
 * dot = a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
 *
 * First term uses fld_arithp (FLD+FMULP). Second and third terms use
 * FMULL [mem] + FADDP — the arith_faddp target.
 */
static bench_ns_t bench_dot3_mem(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a0 = 1.0, a1 = 2.0, a2 = 3.0;
    volatile double b0 = 4.0, b1 = 5.0, b2 = 6.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            /* Term 0: a0*b0 (use fld+fmulp = fld_arithp fusion) */
            "fldl  %1\n\t" /* ST(0) = a0 */
            "fmull %4\n\t" /* ST(0) = a0*b0 = 4 */
            /* Term 1: + a1*b1 (fmull [mem] + faddp = arith_faddp) */
            "fldl  %2\n\t" /* ST(0) = a1, ST(1) = 4 */
            "fmull %5\n\t" /* ST(0) = a1*b1 = 10, ST(1) = 4 -- arith_faddp */
            "faddp\n\t"    /* ST(0) = 14 */
            /* Term 2: + a2*b2 (fmull [mem] + faddp = arith_faddp) */
            "fldl  %3\n\t" /* ST(0) = a2, ST(1) = 14 */
            "fmull %6\n\t" /* ST(0) = a2*b2 = 18, ST(1) = 14 -- arith_faddp */
            "faddp\n\t"    /* ST(0) = 32 */
            "fstpl %0\n"
            : "=m"(r)
            : "m"(a0), "m"(a1), "m"(a2), "m"(b0), "m"(b1), "m"(b2));
    return bench_now_ns() - start;
}

/*
 * Scenario 4: Consecutive FMUL+FADDP pairs from pre-loaded stack values.
 *
 * Values loaded first, then two multiply-accumulate steps without FLD
 * between them.
 */
static bench_ns_t bench_consecutive_madd(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 2.0, b = 3.0, c = 5.0, d = 7.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl  %1\n\t"            /* ST(0)=a=2 */
            "fldl  %2\n\t"            /* ST(0)=b=3, ST(1)=2 */
            "fldl  %3\n\t"            /* ST(0)=c=5, ST(1)=3, ST(2)=2 */
            "fmul  %%st(1), %%st\n\t" /* ST(0)=5*3=15, ST(1)=3, ST(2)=2 -- arith_faddp */
            "faddp\n\t"               /* ST(0)=3+15=18, ST(1)=2 */
            "fldl  %4\n\t"            /* ST(0)=d=7, ST(1)=18, ST(2)=2 */
            "fmul  %%st(1), %%st\n\t" /* ST(0)=7*18=126, ST(1)=18, ST(2)=2 -- arith_faddp */
            "faddp\n\t"               /* ST(0)=18+126=144, ST(1)=2 */
            "fstpl %0\n\t"
            "fstp  %%st(0)\n"
            : "=m"(r)
            : "m"(a), "m"(b), "m"(c), "m"(d));
    return bench_now_ns() - start;
}

/*
 * Scenario 5: FMUL mem + FSUBP — multiply-subtract with memory operand.
 */
static bench_ns_t bench_msub_mem(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 10.0, b = 3.0, c = 2.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl  %1\n\t" /* ST(0)=a=10 */
            "fldl  %2\n\t" /* ST(0)=b=3, ST(1)=10 */
            "fmull %3\n\t" /* ST(0)=3*2=6, ST(1)=10 -- arith_faddp target */
            "fsubp\n\t"    /* GAS fsubp = Intel fsubrp: ST(0)-ST(1)=6-10=-4 */
            "fstpl %0\n"
            : "=m"(r)
            : "m"(a), "m"(b), "m"(c));
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"madd_mem", bench_madd_mem}, {"madd_deep_reg", bench_madd_deep_reg},
        {"dot3_mem", bench_dot3_mem}, {"consecutive_madd", bench_consecutive_madd},
        {"msub_mem", bench_msub_mem},
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
