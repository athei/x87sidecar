/*
 * bench_tag_batch.c -- Benchmarks for OPT-D2 batched tag word updates.
 *
 * Measures the performance impact of deferring pop tag updates to run
 * boundaries. Focus on sequences that generate multiple standalone pops
 * (the path improved by OPT-D2).
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 1000000
#define RUNS 5

/* 4 consecutive pops via FADDP.
 * Before OPT-D2: 4 * 6 = 24 tag instructions inline.
 * After OPT-D2:  4 * 6 = 24 tag instructions at flush (same per-slot cost,
 *                but savings from push-pop cancellation and deferred emission). */
static bench_ns_t bench_pop_chain_4(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"
            "fld1\n\t"
            "fld1\n\t"
            "fld1\n\t"
            "fld1\n\t"
            "faddp\n\t"
            "faddp\n\t"
            "faddp\n\t"
            "faddp\n\t"
            "fstpl %0\n"
            : "=m"(r));
    return bench_now_ns() - start;
}

/* 2 consecutive pops (common case: binary op then store). */
static bench_ns_t bench_pop_chain_2(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"
            "fld1\n\t"
            "fld1\n\t"
            "faddp\n\t"
            "faddp\n\t"
            "fstpl %0\n"
            : "=m"(r));
    return bench_now_ns() - start;
}

/* Alternating push-pop (baseline, already optimized by OPT-D push-pop cancellation). */
static bench_ns_t bench_cancel_chain(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"
            "fld1\n\t"
            "faddp\n\t"
            "fld1\n\t"
            "faddp\n\t"
            "fld1\n\t"
            "faddp\n\t"
            "fld1\n\t"
            "faddp\n\t"
            "fstpl %0\n"
            : "=m"(r));
    return bench_now_ns() - start;
}

/* Realistic a*b + c pattern with mixed push-pop cancellation and standalone pops. */
static bench_ns_t bench_mixed_arith(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 3.0, b = 4.0, c = 5.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl %1\n\t"
            "fldl %2\n\t"
            "fmulp\n\t"
            "fldl %3\n\t"
            "faddp\n\t"
            "fstpl %0\n"
            : "=m"(r)
            : "m"(a), "m"(b), "m"(c));
    return bench_now_ns() - start;
}

/* 6 consecutive pops (stress test — maximum batching benefit). */
static bench_ns_t bench_deep_pop_6(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"
            "fld1\n\t"
            "fld1\n\t"
            "fld1\n\t"
            "fld1\n\t"
            "fld1\n\t"
            "fld1\n\t"
            "faddp\n\t"
            "faddp\n\t"
            "faddp\n\t"
            "faddp\n\t"
            "faddp\n\t"
            "faddp\n\t"
            "fstpl %0\n"
            : "=m"(r));
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"pop_chain_4", bench_pop_chain_4},   {"pop_chain_2", bench_pop_chain_2},
        {"cancel_chain", bench_cancel_chain}, {"mixed_arith", bench_mixed_arith},
        {"deep_pop_6", bench_deep_pop_6},
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
