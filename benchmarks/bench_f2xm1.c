/*
 * bench_f2xm1.c — F2XM1 throughput.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#define TIMES 2000000
#define RUNS  5

static clock_t bench_f2xm1_small(void) {
    volatile double a = 0.125;
    volatile double r;
    clock_t start = clock();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile (
            "fldl  %1\n\t"
            "f2xm1\n\t"
            "fstpl %0\n\t"
            : "=m"(r) : "m"(a) : "st");
    }
    return clock() - start;
}

int main(void) {
    clock_t sum = 0;
    for (int r = 0; r < RUNS; r++) sum += bench_f2xm1_small();
    printf("BENCH %-15s %lu\n", "f2xm1_small", (unsigned long)(sum / RUNS));
    return 0;
}
