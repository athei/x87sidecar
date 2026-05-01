/*
 * bench_fxsave.c — FXSAVE throughput.
 *
 * Loop pushes 8 doubles then fxsaves to a 512-byte buffer.  Unlike
 * fnsave, fxsave does NOT reinitialize the FPU after writing, so the
 * stack is re-cleared with fninit each iter to keep the body symmetric
 * across runs.
 *
 * fxsave is routed to stock translate_insn (we return nullopt; stub
 * abs-jumps to STASH).  Bench measures the full stock-translated path
 * under our loader.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include "bench_timing.h"

#define TIMES 2000000
#define RUNS  5

static bench_ns_t bench_fxsave_loop(void) {
    uint8_t buf[512] __attribute__((aligned(16))) = {0};
    static const double v[8] = {1.0, 2.5, -3.5, 4.0, -5.0, 6.5, 7.25, -8.125};

    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile (
            "fninit\n\t"
            "fldl %1\n\t"
            "fldl %2\n\t"
            "fldl %3\n\t"
            "fldl %4\n\t"
            "fldl %5\n\t"
            "fldl %6\n\t"
            "fldl %7\n\t"
            "fldl %8\n\t"
            "fxsave %0"
            : "=m"(buf)
            : "m"(v[0]), "m"(v[1]), "m"(v[2]), "m"(v[3]),
              "m"(v[4]), "m"(v[5]), "m"(v[6]), "m"(v[7])
            : "memory");
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct { const char *name; bench_ns_t (*fn)(void); } benches[] = {
        {"fxsave_loop", bench_fxsave_loop},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        bench_ns_t sum = 0;
        for (int r = 0; r < RUNS; r++) sum += benches[i].fn();
        printf("BENCH %-15s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
