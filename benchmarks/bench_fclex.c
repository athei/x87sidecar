/*
 * bench_fclex.c — FNCLEX throughput.
 *
 * Single shape: tight loop of fnclex.  Each iteration does the minimum
 * work — clearing the exception flags via the AND-mask 0x7F00.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#define TIMES 2000000
#define RUNS  5

static clock_t bench_fclex_clean(void) {
    clock_t start = clock();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile ("fnclex");
    }
    return clock() - start;
}

int main(void) {
    struct { const char *name; clock_t (*fn)(void); } benches[] = {
        {"fclex_clean", bench_fclex_clean},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        clock_t sum = 0;
        for (int r = 0; r < RUNS; r++) sum += benches[i].fn();
        printf("BENCH %-15s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
