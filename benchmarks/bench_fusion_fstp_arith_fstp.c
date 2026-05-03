/*
 * bench_fusion_fstp_arith_fstp.c -- Benchmark for fstp_arith_fstp 3-op fusion.
 *
 * Pattern: FSTP m32 + FADD m32 + FSTP m32 (~137 M execs/pass in TurtleWoW).
 * Single-op was 3 separate stack-touching ops (~37 ARM); fused is a single
 * load-load-fop-store-store pair with a batched 2-pop (~22 ARM).
 */
#include <stdint.h>
#include <stdio.h>

#include "bench_timing.h"

#define TIMES 1000000
#define RUNS 5

static bench_ns_t bench_fstp_fadd_fstp_m32(void) {
    bench_ns_t start = bench_now_ns();
    volatile float src = 0.5f;
    volatile float d1;
    volatile float d2;
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fld1\n\t"
            "fld1\n\t fadd\n\t" /* push 2.0 */
            "fld1\n\t"          /* push 1.0 → ST(0)=1, ST(1)=2, ST(2)=2 */
            "fstps %0\n\t"      /* [d1]=1.0, pop → ST(0)=2 */
            "fadds %2\n\t"      /* ST(0) = 2 + 0.5 = 2.5 */
            "fstps %1\n\t"      /* [d2]=2.5, pop */
            "fstp %%st(0)\n\t"  /* drain remaining */
            : "=m"(d1), "=m"(d2)
            : "m"(src)
            : "memory");
    }
    return bench_now_ns() - start;
}

static bench_ns_t bench_fstp_fmul_fstp_m32(void) {
    bench_ns_t start = bench_now_ns();
    volatile float src = 2.0f;
    volatile float d1;
    volatile float d2;
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fld1\n\t fld1\n\t fadd\n\t" /* 2.0 */
            "fld1\n\t"
            "fstps %0\n\t"
            "fmuls %2\n\t"
            "fstps %1\n\t"
            "fstp %%st(0)\n\t"
            : "=m"(d1), "=m"(d2)
            : "m"(src)
            : "memory");
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"fstp_fadd_fstp_m32", bench_fstp_fadd_fstp_m32},
        {"fstp_fmul_fstp_m32", bench_fstp_fmul_fstp_m32},
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
