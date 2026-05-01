/*
 * bench_fyl2xp1.c — FYL2XP1 throughput.  fld y; fld x; fyl2xp1; fstpl r.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 2000000
#define RUNS 5

static bench_ns_t bench_fyl2xp1(void) {
    volatile double y = 1.0, x = 0.5;
    volatile double r;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fldl  %1\n\t"
            "fldl  %2\n\t"
            "fyl2xp1\n\t"
            "fstpl %0\n\t"
            : "=m"(r)
            : "m"(y), "m"(x)
            : "st");
    }
    return bench_now_ns() - start;
}

int main(void) {
    bench_ns_t sum = 0;
    for (int r = 0; r < RUNS; r++)
        sum += bench_fyl2xp1();
    printf("BENCH %-15s %lu\n", "fyl2xp1", (unsigned long)(sum / RUNS));
    return 0;
}
