/*
 * bench_fnop.c — FNOP throughput.
 *
 * x87 no-op (D9 D0).  Our handler emits zero ARM instructions for the
 * body but keeps the cache run alive (boundary-flush invariant + run
 * continuity for surrounding ops).  Tight loop measures per-call
 * overhead.
 */
#include <stdint.h>
#include <stdio.h>

#include "bench_timing.h"

#define TIMES 2000000
#define RUNS 5

static bench_ns_t bench_fnop(void) {
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(".byte 0xD9, 0xD0");
    }
    return bench_now_ns() - start;
}

int main(void) {
    bench_fnop(); /* warmup: discard, JIT translates on first call */
    bench_ns_t sum = 0;
    for (int r = 0; r < RUNS; r++)
        sum += bench_fnop();
    printf("BENCH %-15s %lu\n", "fnop", (unsigned long)(sum / RUNS));
    return 0;
}
