/*
 * bench_fdecstp.c — FDECSTP throughput.
 *
 * fdecstp alone unbalances the stack, so the loop pairs it with fincstp.
 * Steady state: TOP returns to its starting value each iteration; no
 * cumulative push/pop pressure.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 2000000
#define RUNS 5

static bench_ns_t bench_fdecstp_pair(void) {
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fdecstp\n\t"
            "fincstp\n\t" ::
                : "st");
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"fdecstp_pair", bench_fdecstp_pair},
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
