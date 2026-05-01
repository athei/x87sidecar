/*
 * bench_fxrstor.c — FXRSTOR throughput.
 *
 * Loop loads the same 512-byte FPU+SSE state every iteration.  We build
 * the source buffer outside the timed loop via fxsave so its byte format
 * matches what Apple's Rosetta expects (the internal f80 representation
 * does not match Intel's spec — see test_fxrstor.c for the reasoning).
 *
 * fxrstor is routed to stock translate_insn (we return nullopt; stub
 * abs-jumps to STASH).  Bench measures the full stock-translated path
 * under our loader.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "bench_timing.h"

#define TIMES 2000000
#define RUNS  5

static bench_ns_t bench_fxrstor_loop(void) {
    /* Produce a valid 512-byte fxsave buffer once: push 8 doubles then
     * fxsave.  The body of the timed loop just fxrstors from it. */
    uint8_t buf[512] __attribute__((aligned(16))) = {0};
    static const double v[8] = {1.0, 2.5, -3.5, 1e10, -1e-10, 42.0, -7.5, 0.5};

    __asm__ volatile (
        ".byte 0xDB, 0xE3\n\t"        /* fninit */
        "fldl %1\n\t" "fldl %2\n\t" "fldl %3\n\t" "fldl %4\n\t"
        "fldl %5\n\t" "fldl %6\n\t" "fldl %7\n\t" "fldl %8\n\t"
        "fxsave %0"
        : "=m"(buf)
        : "m"(v[0]), "m"(v[1]), "m"(v[2]), "m"(v[3]),
          "m"(v[4]), "m"(v[5]), "m"(v[6]), "m"(v[7])
        : "memory");

    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile ("fxrstor %0" : : "m"(buf) : "memory");
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct { const char *name; bench_ns_t (*fn)(void); } benches[] = {
        {"fxrstor_loop", bench_fxrstor_loop},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        bench_ns_t sum = 0;
        for (int r = 0; r < RUNS; r++) sum += benches[i].fn();
        printf("BENCH %-15s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
