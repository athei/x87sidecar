/*
 * bench_fprem.c — FPREM throughput.  fld y; fld x; fprem; fstpl r;
 * ffree st(0); fincstp.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#define TIMES 2000000
#define RUNS  5

static clock_t bench_fprem(void) {
    volatile double x = 5.0, y = 3.0;
    volatile double r;
    clock_t start = clock();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile (
            "fldl  %1\n\t"
            "fldl  %2\n\t"
            "fprem\n\t"
            "fstpl %0\n\t"
            "ffree %%st(0)\n\t"
            "fincstp\n\t"
            : "=m"(r) : "m"(y), "m"(x) : "st");
    }
    return clock() - start;
}

int main(void) {
    clock_t sum = 0;
    for (int r = 0; r < RUNS; r++) sum += bench_fprem();
    printf("BENCH %-15s %lu\n", "fprem", (unsigned long)(sum / RUNS));
    return 0;
}
