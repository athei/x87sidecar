/*
 * bench_ffree.c — FFREE throughput.
 *
 * Loop pattern: fld + ffree ST(0) so the stack stays balanced (the fld
 * makes the slot valid, ffree marks it empty, and the slot is overwritten
 * by the next iteration's fld).
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 2000000
#define RUNS 5

static bench_ns_t bench_ffree_st0(void) {
    volatile double x = 1.5;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fldl %0\n\t"
            "ffree %%st(0)\n\t"
            "fincstp\n\t" /* drop the (now-empty) slot from the architectural view */
            :
            : "m"(x)
            : "st");
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"ffree_st0", bench_ffree_st0},
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
