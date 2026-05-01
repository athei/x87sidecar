/*
 * bench_finit.c — FNINIT throughput.
 *
 * Single shape: tight loop of FNINIT (DB E3, no WAIT prefix).  Each
 * iteration emits 3 stores to control/status/tag word.  Single-instr
 * loop is prologue-bound on the JIT side (per-instr cache prologue/
 * epilogue dominates), so don't expect a >1x win here — finit is rare
 * in real workloads and the win comes from never falling through to
 * stock for it, which preserves cache integrity for surrounding code.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 2000000
#define RUNS 5

static bench_ns_t bench_finit_loop(void) {
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(".byte 0xDB, 0xE3");
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"finit_loop", bench_finit_loop},
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
