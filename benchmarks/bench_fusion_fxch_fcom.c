/*
 * bench_fusion_fxch_fcom.c -- Benchmark for fxch_fcom 2-op + fxch_fcom_fstsw
 * 3-op fusions (Pattern B + C, ~1.25 B execs/pass in TurtleWoW).
 *
 * Pattern: FXCH ST(N) + FCOM/FCOMP m32/m64 [+ FNSTSW AX].  FXCH is absorbed
 * as a compile-time perm swap (OPT-G); the body is the existing
 * fcom_fstsw/translate_fcom emit, just reading from the swapped slot.
 */
#include <stdint.h>
#include <stdio.h>

#include "bench_timing.h"

#define TIMES 1000000
#define RUNS 5

/* fxch ST(1) + fcom_m32 (no fstsw) — 2-op fusion.
 * Setup: stack pre-loaded with two values, fxch swaps ST(0)/ST(1), then
 * fcom compares ST(0) (= old ST(1)) vs memory.
 */
static bench_ns_t bench_fxch_st1_fcom_m32(void) {
    bench_ns_t start = bench_now_ns();
    volatile float cmp = 1.5f;
    /* Prime: push two values onto x87 stack so fxch ST(1) is meaningful. */
    __asm__ volatile("fld1\n\t fld1\n\t fadd\n\t fld1\n\t" : : :);
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fxch %%st(1)\n\t"
            "fcoms %0\n\t"
            :
            : "m"(cmp));
    }
    __asm__ volatile("fstp %%st(0)\n\t fstp %%st(0)\n\t" : : :);
    return bench_now_ns() - start;
}

/* fxch ST(2) + fcomp_m32 + fstsw_ax — 3-op fusion.
 * Setup: 3 values on stack, fxch swaps with ST(2), fcomp compares + pops,
 * fstsw captures CC bits.  Loop refills each iter to keep stack non-empty.
 */
static bench_ns_t bench_fxch_st2_fcomp_m32_fstsw(void) {
    bench_ns_t start = bench_now_ns();
    volatile float cmp = 1.5f;
    volatile uint16_t sw;
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fld1\n\t fld1\n\t fadd\n\t" /* ST(0) = 2.0 */
            "fld1\n\t"                   /* push 1.0 -> ST(0)=1, ST(1)=2 */
            "fld1\n\t"                   /* push 1.0 -> ST(0)=1, ST(1)=1, ST(2)=2 */
            "fxch %%st(2)\n\t"           /* swap ST(0)/ST(2) -> ST(0)=2, ST(2)=1 */
            "fcomps %2\n\t"              /* compare 2.0 vs cmp, pop */
            "fnstsw %%ax\n\t"
            "movw %%ax, %1\n\t"
            "fstp %%st(0)\n\t fstp %%st(0)\n\t" /* drain remaining */
            : "+m"(cmp)
            : "m"(sw), "m"(cmp)
            : "ax", "memory");
    }
    return bench_now_ns() - start;
}

/* fxch ST(3) + fcom_m32 + fstsw_ax — 3-op fusion, deeper depth. */
static bench_ns_t bench_fxch_st3_fcom_m32_fstsw(void) {
    bench_ns_t start = bench_now_ns();
    volatile float cmp = 1.5f;
    volatile uint16_t sw;
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fld1\n\t fld1\n\t fld1\n\t fld1\n\t" /* 4 values */
            "fxch %%st(3)\n\t"
            "fcoms %1\n\t"
            "fnstsw %%ax\n\t"
            "movw %%ax, %0\n\t"
            "fstp %%st(0)\n\t fstp %%st(0)\n\t fstp %%st(0)\n\t fstp %%st(0)\n\t"
            : "=m"(sw)
            : "m"(cmp)
            : "ax");
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"fxch_st1_fcom_m32", bench_fxch_st1_fcom_m32},
        {"fxch_st2_fcomp_m32_fstsw", bench_fxch_st2_fcomp_m32_fstsw},
        {"fxch_st3_fcom_m32_fstsw", bench_fxch_st3_fcom_m32_fstsw},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        benches[i].fn(); /* warmup */
        bench_ns_t sum = 0;
        for (int r = 0; r < RUNS; r++)
            sum += benches[i].fn();
        printf("BENCH %s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
