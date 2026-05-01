/*
 * bench_fcos.c — FCOS throughput.  Same shape as bench_fsin.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 2000000
#define RUNS 5

static bench_ns_t bench_fcos_small(void) {
    volatile double a = 0.125;
    volatile double r;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fldl  %1\n\t"
            "fcos\n\t"
            "fstpl %0\n\t"
            : "=m"(r)
            : "m"(a)
            : "st");
    }
    return bench_now_ns() - start;
}

static bench_ns_t bench_fcos_midrange(void) {
    volatile double a = 0.7853981633974483; /* ≈ M_PI/4 */
    volatile double r;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fldl  %1\n\t"
            "fcos\n\t"
            "fstpl %0\n\t"
            : "=m"(r)
            : "m"(a)
            : "st");
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"fcos_small", bench_fcos_small},
        {"fcos_midrange", bench_fcos_midrange},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        benches[i].fn(); /* warmup: discard, JIT translates on first call */
        bench_ns_t sum = 0;
        for (int r = 0; r < RUNS; r++)
            sum += benches[i].fn();
        printf("BENCH %-15s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
