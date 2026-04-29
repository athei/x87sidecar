/*
 * bench_fincstp.c — FINCSTP throughput.
 *
 * fincstp alone unbalances the stack, so the loop pairs it with fdecstp.
 * Steady state: TOP returns to its starting value each iteration.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#define TIMES 2000000
#define RUNS  5

static clock_t bench_fincstp_pair(void) {
    clock_t start = clock();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile (
            "fincstp\n\t"
            "fdecstp\n\t"
            ::: "st");
    }
    return clock() - start;
}

int main(void) {
    struct { const char *name; clock_t (*fn)(void); } benches[] = {
        {"fincstp_pair", bench_fincstp_pair},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        clock_t sum = 0;
        for (int r = 0; r < RUNS; r++) sum += benches[i].fn();
        printf("BENCH %-15s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
