/*
 * bench_fxch.c -- Benchmark for FXCH register exchange (OPT-G: rename).
 *
 * Measures the cost of FXCH in isolation and in common patterns:
 *   - Bare FXCH ST(1) (the most common form)
 *   - FXCH ST(2) / ST(3) (deeper exchanges)
 *   - Consecutive FXCHs (compiler-generated reordering)
 *   - FXCH + FADDP (common arithmetic pattern, also covered by fusion)
 *   - FXCH + FSTP (discard pattern, also covered by fusion)
 *   - Double FXCH (swap-back, should be free with deferred FXCH)
 *
 * Compare with X87_DISABLE_DEFERRED_FXCH=1 to measure the benefit.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 1000000
#define RUNS 5

/* Bare FXCH ST(1) -- the rename sweet spot.
 * Push two values, exchange, pop both.  The FXCH itself should be free. */
static bench_ns_t bench_fxch_st1(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"
            "fld1\n\t fld1\n\t faddp\n\t" /* ST(0)=2, ST(1)=1 */
            "fxch %%st(1)\n\t"            /* swap: ST(0)=1, ST(1)=2 */
            "fstpl %0\n\t"
            "fstp %%st(0)\n"
            : "=m"(r));
    return bench_now_ns() - start;
}

/* FXCH ST(2) -- deeper exchange */
static bench_ns_t bench_fxch_st2(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"                                       /* 1 */
            "fld1\n\t fld1\n\t faddp\n\t"                    /* 2 */
            "fld1\n\t fld1\n\t faddp\n\t fld1\n\t faddp\n\t" /* 3 */
            /* ST(0)=3, ST(1)=2, ST(2)=1 */
            "fxch %%st(2)\n\t" /* ST(0)=1, ST(1)=2, ST(2)=3 */
            "fstpl %0\n\t"
            "fstp %%st(0)\n\t"
            "fstp %%st(0)\n"
            : "=m"(r));
    return bench_now_ns() - start;
}

/* FXCH ST(3) -- even deeper */
static bench_ns_t bench_fxch_st3(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"
            "fld1\n\t fld1\n\t faddp\n\t"
            "fld1\n\t fld1\n\t faddp\n\t fld1\n\t faddp\n\t"
            "fld1\n\t fld1\n\t faddp\n\t fld1\n\t faddp\n\t fld1\n\t faddp\n\t"
            /* ST(0)=4, ST(1)=3, ST(2)=2, ST(3)=1 */
            "fxch %%st(3)\n\t"
            "fstpl %0\n\t"
            "fstp %%st(0)\n\t fstp %%st(0)\n\t fstp %%st(0)\n"
            : "=m"(r));
    return bench_now_ns() - start;
}

/* Double FXCH -- swap and swap back (no-op with rename) */
static bench_ns_t bench_fxch_double(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"
            "fld1\n\t fld1\n\t faddp\n\t" /* ST(0)=2, ST(1)=1 */
            "fxch %%st(1)\n\t"            /* swap */
            "fxch %%st(1)\n\t"            /* swap back */
            "fstpl %0\n\t"
            "fstp %%st(0)\n"
            : "=m"(r));
    return bench_now_ns() - start;
}

/* Triple consecutive FXCH -- compiler-generated reordering pattern */
static bench_ns_t bench_fxch_triple(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"
            "fld1\n\t fld1\n\t faddp\n\t"
            "fld1\n\t fld1\n\t faddp\n\t fld1\n\t faddp\n\t"
            /* ST(0)=3, ST(1)=2, ST(2)=1 */
            "fxch %%st(1)\n\t"
            "fxch %%st(2)\n\t"
            "fxch %%st(1)\n\t"
            "fstpl %0\n\t"
            "fstp %%st(0)\n\t fstp %%st(0)\n"
            : "=m"(r));
    return bench_now_ns() - start;
}

/* FXCH + FADDP -- common arithmetic pattern */
static bench_ns_t bench_fxch_add(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"
            "fld1\n\t fld1\n\t faddp\n\t" /* ST(0)=2, ST(1)=1 */
            "fxch %%st(1)\n\t"
            "faddp\n\t" /* 2+1=3 */
            "fstpl %0\n"
            : "=m"(r));
    return bench_now_ns() - start;
}

/* FXCH + FSTP -- discard after swap pattern */
static bench_ns_t bench_fxch_fstp(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"
            "fld1\n\t fld1\n\t faddp\n\t" /* ST(0)=2, ST(1)=1 */
            "fxch %%st(1)\n\t"
            "fstp %%st(0)\n\t" /* discard swapped top */
            "fstpl %0\n"
            : "=m"(r));
    return bench_now_ns() - start;
}

/* Repeated FXCH in a tight loop -- stress test */
static bench_ns_t bench_fxch_loop(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"
            "fld1\n\t fld1\n\t faddp\n\t" /* ST(0)=2, ST(1)=1 */
            "fxch %%st(1)\n\t"
            "fxch %%st(1)\n\t"
            "fxch %%st(1)\n\t"
            "fxch %%st(1)\n\t"
            "fxch %%st(1)\n\t"
            "fxch %%st(1)\n\t"
            "fxch %%st(1)\n\t"
            "fxch %%st(1)\n\t"
            "fstpl %0\n\t"
            "fstp %%st(0)\n"
            : "=m"(r));
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"fxch_st1", bench_fxch_st1},       {"fxch_st2", bench_fxch_st2},
        {"fxch_st3", bench_fxch_st3},       {"fxch_double", bench_fxch_double},
        {"fxch_triple", bench_fxch_triple}, {"fxch_add", bench_fxch_add},
        {"fxch_fstp", bench_fxch_fstp},     {"fxch_loop_8x", bench_fxch_loop},
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
