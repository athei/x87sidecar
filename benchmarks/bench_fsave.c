/*
 * bench_fsave.c — FSAVE / FNSAVE throughput.
 *
 * fsave is inline-handled by our JIT (the f64→f80 conversion of the 8 ST
 * slots is the bulk of its emit; ~22 insns per slot).  Two sub-benchmarks:
 *   fsave_solo:        bare `for(TIMES) fnsave [buf]` — fnsave reinitializes
 *                      the FPU after writing per the SDM, so each iter
 *                      starts fresh on an empty stack.  Honest measure of
 *                      fnsave on an empty stack (no f80 conversion work
 *                      needed for tagged-empty slots, so fast).
 *   fsave_with_stack:  8×fldl + fnsave per iter — the realistic case where
 *                      fsave actually has 8 valid f64 values to convert
 *                      to f80.  This is the loop body that exercises the
 *                      8-slot conversion path.  Note: 8 of the 9 ops are
 *                      our inline-JIT'd fld; the speedup vs stock is real
 *                      for the whole body but most comes from the flds
 *                      (see feedback_bench_name_lies.md).
 */
#include <stdint.h>
#include <stdio.h>

#include "bench_timing.h"

#define TIMES 2000000
#define RUNS 5

static bench_ns_t bench_fsave_solo(void) {
    uint8_t buf[108] __attribute__((aligned(16))) = {0};
    /* fnsave auto-reinitializes the FPU, so each iter starts with an
     * empty tag word (all 8 slots tagged kEmpty).  The 8-slot ST
     * conversion path emits f80 zero bytes for empty slots — fast. */
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile("fnsave %0" : "=m"(buf) : : "memory");
    }
    return bench_now_ns() - start;
}

static bench_ns_t bench_fsave_with_stack(void) {
    uint8_t buf[108] __attribute__((aligned(16))) = {0};
    static const double v[8] = {1.0, 2.5, -3.5, 4.0, -5.0, 6.5, 7.25, -8.125};

    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        /* Re-fill stack: 8 fld's bring the stack back to fully populated.
         * %0..%7 are v[0..7] inputs; %8 is buf output.  fnsave clears
         * the FPU after writing, so each iter starts fresh. */
        __asm__ volatile(
            "fldl %1\n\t"
            "fldl %2\n\t"
            "fldl %3\n\t"
            "fldl %4\n\t"
            "fldl %5\n\t"
            "fldl %6\n\t"
            "fldl %7\n\t"
            "fldl %8\n\t"
            "fnsave %0"
            : "=m"(buf)
            : "m"(v[0]), "m"(v[1]), "m"(v[2]), "m"(v[3]), "m"(v[4]), "m"(v[5]), "m"(v[6]), "m"(v[7])
            : "memory");
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"fsave_solo", bench_fsave_solo},
        {"fsave_with_stack", bench_fsave_with_stack},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        benches[i].fn(); /* warmup: discard, JIT translates on first call */
        bench_ns_t sum = 0;
        for (int r = 0; r < RUNS; r++)
            sum += benches[i].fn();
        printf("BENCH %-20s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
