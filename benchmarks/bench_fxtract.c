/*
 * bench_fxtract.c — FXTRACT throughput.
 *
 * Each iteration: fld + fxtract + 2× fstpl (drains both results).
 * Two shapes: a normal positive value and a negative value (exercises
 * the sign-preserving path on the significand).
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#define TIMES 2000000
#define RUNS  5

static clock_t bench_fxtract_pos(void) {
    volatile double x = 6.0;
    volatile double sig, exp_v;
    clock_t start = clock();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile (
            "fldl   %2\n\t"
            "fxtract\n\t"
            "fstpl  %0\n\t"
            "fstpl  %1\n\t"
            : "=m"(sig), "=m"(exp_v) : "m"(x) : "st");
    }
    return clock() - start;
}

static clock_t bench_fxtract_neg(void) {
    volatile double x = -1024.0;
    volatile double sig, exp_v;
    clock_t start = clock();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile (
            "fldl   %2\n\t"
            "fxtract\n\t"
            "fstpl  %0\n\t"
            "fstpl  %1\n\t"
            : "=m"(sig), "=m"(exp_v) : "m"(x) : "st");
    }
    return clock() - start;
}

int main(void) {
    struct { const char *name; clock_t (*fn)(void); } benches[] = {
        {"fxtract_pos", bench_fxtract_pos},
        {"fxtract_neg", bench_fxtract_neg},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        clock_t sum = 0;
        for (int r = 0; r < RUNS; r++) sum += benches[i].fn();
        printf("BENCH %-15s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
