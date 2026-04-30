/*
 * bench_fsave.c — FSAVE throughput.
 *
 * Loop pushes 8 doubles then fsaves to a buffer (which also re-initializes
 * the FPU per the SDM, so the loop body's pushes start fresh each time).
 * Body emits ~26 instructions for the env header + 8 unrolled f64->f80
 * conversions (~22 insns each) plus 8 emit_load_st calls.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#define TIMES 2000000
#define RUNS  5

static clock_t bench_fsave_loop(void) {
    uint8_t buf[108] __attribute__((aligned(16))) = {0};
    static const double v[8] = {1.0, 2.5, -3.5, 4.0, -5.0, 6.5, 7.25, -8.125};

    clock_t start = clock();
    for (int i = 0; i < TIMES; i++) {
        /* Re-fill stack: 8 fld's bring the stack back to fully populated.
         * %0..%7 are v[0..7] inputs; %8 is buf output.  fnsave clears
         * the FPU after writing, so each iter starts fresh. */
        __asm__ volatile (
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
            : "m"(v[0]), "m"(v[1]), "m"(v[2]), "m"(v[3]),
              "m"(v[4]), "m"(v[5]), "m"(v[6]), "m"(v[7])
            : "memory");
    }
    return clock() - start;
}

int main(void) {
    struct { const char *name; clock_t (*fn)(void); } benches[] = {
        {"fsave_loop", bench_fsave_loop},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        clock_t sum = 0;
        for (int r = 0; r < RUNS; r++) sum += benches[i].fn();
        printf("BENCH %-15s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
