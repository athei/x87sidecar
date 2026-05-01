/*
 * bench_fxsave.c — FXSAVE throughput.
 *
 * Two sub-benchmarks:
 *   fxsave_solo:        bare `for(TIMES) fxsave [buf]` after one fninit.
 *                       Honest measure of fxsave alone.  fxsave is routed
 *                       to stock translate_insn (we return nullopt; stub
 *                       abs-jumps to STASH), so JIT and --disable-hook
 *                       should be at parity here.
 *   fxsave_with_stack:  fninit + 8×fldl + fxsave per iter.  Measures
 *                       fxsave in a realistic context with an 8-deep
 *                       stack to save.  Headline speedup here is
 *                       dominated by our inline-JIT'd fld + fninit, NOT
 *                       fxsave itself — see feedback_bench_name_lies.md.
 */
#include <stdint.h>
#include <stdio.h>

#include "bench_timing.h"

#define TIMES 2000000
#define RUNS 5

static bench_ns_t bench_fxsave_solo(void) {
    uint8_t buf[512] __attribute__((aligned(16))) = {0};
    /* One-time init: fxsave does not modify FPU state, so an fninit'd
     * stack stays empty across iterations.  The body measures only the
     * fxsave instruction. */
    __asm__ volatile("fninit" ::: "memory");

    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile("fxsave %0" : "=m"(buf) : : "memory");
    }
    return bench_now_ns() - start;
}

static bench_ns_t bench_fxsave_with_stack(void) {
    uint8_t buf[512] __attribute__((aligned(16))) = {0};
    static const double v[8] = {1.0, 2.5, -3.5, 4.0, -5.0, 6.5, 7.25, -8.125};

    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
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
        {"fxsave_solo", bench_fxsave_solo},
        {"fxsave_with_stack", bench_fxsave_with_stack},
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
