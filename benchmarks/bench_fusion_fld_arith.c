/*
 * bench_fusion_fld_arith.c -- Benchmark for the new 2-op fld+arith mem-mem
 * fusion (Pattern D, ~230 M execs/pass for fld_m32+fmul_m32 alone).
 *
 * Each iteration runs `flds m_a; fmuls m_b; nop; fstps m_c` so the run is
 * exactly 2 contiguous x87 ops (gate-blocked from IR), with the fstp in a
 * separate run.  With the `fld_arith` fusion the pair becomes a single
 * load-load-fop-push; without it the pair is two single-ops.
 */
#include <stdint.h>
#include <stdio.h>

#include "bench_timing.h"

#define TIMES 1000000
#define RUNS 5

static bench_ns_t bench_fld_fmul_m32(void) {
    bench_ns_t start = bench_now_ns();
    volatile float a = 3.0f;
    volatile float b = 4.0f;
    volatile float r;
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "flds %1\n\t"
            "fmuls %2\n\t"
            "nop\n\t"
            "fstps %0\n\t"
            : "=m"(r)
            : "m"(a), "m"(b));
    }
    return bench_now_ns() - start;
}

static bench_ns_t bench_fld_fadd_m32(void) {
    bench_ns_t start = bench_now_ns();
    volatile float a = 3.0f;
    volatile float b = 4.0f;
    volatile float r;
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "flds %1\n\t"
            "fadds %2\n\t"
            "nop\n\t"
            "fstps %0\n\t"
            : "=m"(r)
            : "m"(a), "m"(b));
    }
    return bench_now_ns() - start;
}

static bench_ns_t bench_fld_fmul_m64(void) {
    bench_ns_t start = bench_now_ns();
    volatile double a = 3.0;
    volatile double b = 4.0;
    volatile double r;
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fldl %1\n\t"
            "fmull %2\n\t"
            "nop\n\t"
            "fstpl %0\n\t"
            : "=m"(r)
            : "m"(a), "m"(b));
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"fld_fmul_m32", bench_fld_fmul_m32},
        {"fld_fadd_m32", bench_fld_fadd_m32},
        {"fld_fmul_m64", bench_fld_fmul_m64},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        benches[i].fn(); /* warmup */
        bench_ns_t sum = 0;
        for (int r = 0; r < RUNS; r++)
            sum += benches[i].fn();
        printf("BENCH %s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
