/*
 * bench_fptan.c — FPTAN throughput.  fld v; fptan; fstpl one; fstpl tan.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#define TIMES 2000000
#define RUNS  5

static clock_t bench_fptan(void) {
    volatile double v = 0.7853981633974483; /* ≈ M_PI/4 */
    volatile double t, one;
    clock_t start = clock();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile (
            "fldl  %2\n\t"
            "fptan\n\t"
            "fstpl %1\n\t"
            "fstpl %0\n\t"
            : "=m"(t), "=m"(one) : "m"(v) : "st");
    }
    return clock() - start;
}

int main(void) {
    clock_t sum = 0;
    for (int r = 0; r < RUNS; r++) sum += bench_fptan();
    printf("BENCH %-15s %lu\n", "fptan", (unsigned long)(sum / RUNS));
    return 0;
}
