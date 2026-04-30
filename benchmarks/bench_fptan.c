/*
 * bench_fptan.c — FPTAN throughput.  fld v; fptan; fstpl one; fstpl tan.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include "bench_timing.h"

#define TIMES 2000000
#define RUNS  5

static bench_ns_t bench_fptan(void) {
    volatile double v = 0.7853981633974483; /* ≈ M_PI/4 */
    volatile double t, one;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile (
            "fldl  %2\n\t"
            "fptan\n\t"
            "fstpl %1\n\t"
            "fstpl %0\n\t"
            : "=m"(t), "=m"(one) : "m"(v) : "st");
    }
    return bench_now_ns() - start;
}

int main(void) {
    bench_ns_t sum = 0;
    for (int r = 0; r < RUNS; r++) sum += bench_fptan();
    printf("BENCH %-15s %lu\n", "fptan", (unsigned long)(sum / RUNS));
    return 0;
}
