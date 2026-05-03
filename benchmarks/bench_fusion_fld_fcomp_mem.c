/*
 * bench_fusion_fld_fcomp_mem.c -- Benchmark for fld_fcomp 2-op fusion
 * with memory-operand FCOMP (no FSTSW after).
 *
 * Pattern: FLD m32/m64 + FCOMP m32/m64 (compare loaded value vs memory, pop).
 * In a real WoW workload this is the most-executed x87 pattern in the
 * profile (~500 M execs / pass).  The 2-op peephole fuses push+compare+pop
 * into a single FCMP + status_word RMW, avoiding the full inline emit
 * that would do two stack pushes and a separate fcomp pop.
 */
#include <stdint.h>
#include <stdio.h>

#include "bench_timing.h"

#define TIMES 1000000
#define RUNS 5

/* fld m64 + fcomp m32 — measures the most common form. */
static bench_ns_t bench_fld_fcomp_m32(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 3.0;
    volatile float b = 1.5f;
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fldl %0\n\t"   /* push a */
            "fcomps %1\n\t" /* compare ST(0) vs b, pop */
            :
            : "m"(a), "m"(b));
    }
    return bench_now_ns() - start;
}

/* fld m64 + fcomp m64 */
static bench_ns_t bench_fld_fcomp_m64(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 3.0;
    volatile double b = 1.5;
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fldl %0\n\t"
            "fcompl %1\n\t"
            :
            : "m"(a), "m"(b));
    }
    return bench_now_ns() - start;
}

/* fld m32 + fcomp m32 — both single-precision (analyzer-reported hottest). */
static bench_ns_t bench_fld_m32_fcomp_m32(void) {
    bench_ns_t start = bench_now_ns();
    volatile float a = 3.0f;
    volatile float b = 1.5f;
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "flds %0\n\t"
            "fcomps %1\n\t"
            :
            : "m"(a), "m"(b));
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"fld_m64_fcomp_m32", bench_fld_fcomp_m32},
        {"fld_m64_fcomp_m64", bench_fld_fcomp_m64},
        {"fld_m32_fcomp_m32", bench_fld_m32_fcomp_m32},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        benches[i].fn(); /* warmup: discard, JIT translates on first call */
        bench_ns_t sum = 0;
        for (int r = 0; r < RUNS; r++)
            sum += benches[i].fn();
        printf("BENCH %s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
