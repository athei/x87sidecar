/*
 * bench_fscale.c — FSCALE throughput.
 *
 * Each iteration: fld b; fld a; fscale; fstpl r; ffree+fincstp to drop b.
 * Three shapes: small positive k, small negative k, large k (overflow).
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 2000000
#define RUNS 5

static bench_ns_t bench_fscale_pos(void) {
    volatile double a = 1.5, b = 3.0;
    volatile double r;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fldl  %1\n\t"
            "fldl  %2\n\t"
            "fscale\n\t"
            "fstpl %0\n\t"
            "ffree %%st(0)\n\t"
            "fincstp\n\t"
            : "=m"(r)
            : "m"(b), "m"(a)
            : "st");
    }
    return bench_now_ns() - start;
}

static bench_ns_t bench_fscale_neg(void) {
    volatile double a = 7.0, b = -2.0;
    volatile double r;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fldl  %1\n\t"
            "fldl  %2\n\t"
            "fscale\n\t"
            "fstpl %0\n\t"
            "ffree %%st(0)\n\t"
            "fincstp\n\t"
            : "=m"(r)
            : "m"(b), "m"(a)
            : "st");
    }
    return bench_now_ns() - start;
}

static bench_ns_t bench_fscale_overflow(void) {
    /* k far above representable → result is +Inf. */
    volatile double a = 2.0, b = 2000.0;
    volatile double r;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fldl  %1\n\t"
            "fldl  %2\n\t"
            "fscale\n\t"
            "fstpl %0\n\t"
            "ffree %%st(0)\n\t"
            "fincstp\n\t"
            : "=m"(r)
            : "m"(b), "m"(a)
            : "st");
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"fscale_pos", bench_fscale_pos},
        {"fscale_neg", bench_fscale_neg},
        {"fscale_overflow", bench_fscale_overflow},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        bench_ns_t sum = 0;
        for (int r = 0; r < RUNS; r++)
            sum += benches[i].fn();
        printf("BENCH %-15s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
