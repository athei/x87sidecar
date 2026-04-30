/*
 * bench_f2xm1.c — F2XM1 throughput.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include "bench_timing.h"

#define TIMES 2000000
#define RUNS  5

static bench_ns_t bench_f2xm1_small(void) {
    volatile double a = 0.125;
    volatile double r;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile (
            "fldl  %1\n\t"
            "f2xm1\n\t"
            "fstpl %0\n\t"
            : "=m"(r) : "m"(a) : "st");
    }
    return bench_now_ns() - start;
}

int main(void) {
    bench_ns_t sum = 0;
    for (int r = 0; r < RUNS; r++) sum += bench_f2xm1_small();
    printf("BENCH %-15s %lu\n", "f2xm1_small", (unsigned long)(sum / RUNS));
    return 0;
}
