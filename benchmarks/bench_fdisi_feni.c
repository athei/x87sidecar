/*
 * bench_fdisi_feni.c — FDISI / FENI throughput.
 *
 * Both are unconditional NOPs on 80287+; our handler emits nothing.
 * Tight loop measures per-call overhead (cache prologue/epilogue).
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include "bench_timing.h"

#define TIMES 2000000
#define RUNS  5

static bench_ns_t bench_fdisi(void) {
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile (".byte 0xDB, 0xE1");
    }
    return bench_now_ns() - start;
}

static bench_ns_t bench_feni(void) {
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile (".byte 0xDB, 0xE0");
    }
    return bench_now_ns() - start;
}

static bench_ns_t bench_pair(void) {
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile (
            ".byte 0xDB, 0xE1\n\t"   /* fdisi */
            ".byte 0xDB, 0xE0\n\t"); /* feni  */
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct { const char *name; bench_ns_t (*fn)(void); } benches[] = {
        {"fdisi",      bench_fdisi},
        {"feni",       bench_feni},
        {"disi_eni",   bench_pair},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        bench_ns_t sum = 0;
        for (int r = 0; r < RUNS; r++) sum += benches[i].fn();
        printf("BENCH %-15s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
