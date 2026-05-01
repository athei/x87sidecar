/*
 * bench_fstenv.c — FNSTENV throughput.
 *
 * Loop loads/stores the 28-byte env to a stack-local buffer.  9-instruction
 * JIT body (3 LDRH+STR + 3 zero stores).
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 2000000
#define RUNS 5

static bench_ns_t bench_fstenv_loop(void) {
    volatile uint8_t env[28] = {0};
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile("fnstenv %0" : "=m"(env)::"memory");
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"fstenv_loop", bench_fstenv_loop},
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
