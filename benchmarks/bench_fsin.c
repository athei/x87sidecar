/*
 * bench_fsin.c — FSIN throughput.
 *
 * Each iteration: fld a; fsin; fstpl r.  Three input shapes:
 *   small      — small angle near 0 (cheap-path)
 *   midrange   — ~M_PI/4 (typical)
 *   large      — large angle that on stock would trip x87's >=2^63
 *                "out-of-range" guard (we always compute via libm; the
 *                JIT logs the simplification on first hit).
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include "bench_timing.h"

#define TIMES 2000000
#define RUNS  5

static bench_ns_t bench_fsin_small(void) {
    volatile double a = 0.125;
    volatile double r;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile (
            "fldl  %1\n\t"
            "fsin\n\t"
            "fstpl %0\n\t"
            : "=m"(r) : "m"(a) : "st");
    }
    return bench_now_ns() - start;
}

static bench_ns_t bench_fsin_midrange(void) {
    volatile double a = 0.7853981633974483; /* ≈ M_PI/4 */
    volatile double r;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile (
            "fldl  %1\n\t"
            "fsin\n\t"
            "fstpl %0\n\t"
            : "=m"(r) : "m"(a) : "st");
    }
    return bench_now_ns() - start;
}

static bench_ns_t bench_fsin_large(void) {
    /* Far above 2^63 — stock would set C2 and skip; we always compute. */
    volatile double a = 1.0e20;
    volatile double r;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile (
            "fldl  %1\n\t"
            "fsin\n\t"
            "fstpl %0\n\t"
            : "=m"(r) : "m"(a) : "st");
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct { const char *name; bench_ns_t (*fn)(void); } benches[] = {
        {"fsin_small",    bench_fsin_small},
        {"fsin_midrange", bench_fsin_midrange},
        {"fsin_large",    bench_fsin_large},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        bench_ns_t sum = 0;
        for (int r = 0; r < RUNS; r++) sum += benches[i].fn();
        printf("BENCH %-15s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
