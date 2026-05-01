/*
 * bench_fusion_fcom_fstsw.c -- Benchmark for fcom_fstsw fusion.
 * Pattern: FCOM/FCOMP + FNSTSW AX — compare immediately followed by status read.
 * The fusion avoids the separate FNSTSW dispatch.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 1000000
#define RUNS 5

/* FCOM ST(1) + FNSTSW (non-popping compare, register operand) */
static bench_ns_t bench_fcom_st_fstsw(void) {
    bench_ns_t start = bench_now_ns();
    volatile uint16_t sw;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"
            "fld1\n\t fld1\n\t faddp\n\t" /* ST(0)=2, ST(1)=1 */
            "fcom %%st(1)\n\t"
            "fnstsw %%ax\n\t"
            "movw %%ax, %0\n\t"
            "fstp %%st(0)\n\t"
            "fstp %%st(0)\n\t"
            : "=m"(sw)
            :
            : "ax");
    return bench_now_ns() - start;
}

/* FCOML m64 + FNSTSW (memory operand compare with pop) */
static bench_ns_t bench_fcom_m64_fstsw(void) {
    bench_ns_t start = bench_now_ns();
    volatile double cmp = 5.0;
    volatile uint16_t sw;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"
            "fcoml %1\n\t"
            "fnstsw %%ax\n\t"
            "movw %%ax, %0\n\t"
            "fstp %%st(0)\n\t"
            : "=m"(sw)
            : "m"(cmp)
            : "ax");
    return bench_now_ns() - start;
}

/* FCOMP ST(1) + FNSTSW (popping compare, register operand) */
static bench_ns_t bench_fcomp_st_fstsw(void) {
    bench_ns_t start = bench_now_ns();
    volatile uint16_t sw;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"
            "fld1\n\t fld1\n\t faddp\n\t" /* ST(0)=2, ST(1)=1 */
            "fcomp %%st(1)\n\t"           /* compare, pop -> ST(0)=1 */
            "fnstsw %%ax\n\t"
            "movw %%ax, %0\n\t"
            "fstp %%st(0)\n\t"
            : "=m"(sw)
            :
            : "ax");
    return bench_now_ns() - start;
}

/* FCOMPL m64 + FNSTSW (popping compare, memory operand) */
static bench_ns_t bench_fcomp_m64_fstsw(void) {
    bench_ns_t start = bench_now_ns();
    volatile double cmp = 0.5;
    volatile uint16_t sw;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t fld1\n\t faddp\n\t" /* ST(0)=2 */
            "fcompl %1\n\t"
            "fnstsw %%ax\n\t"
            "movw %%ax, %0\n\t"
            : "=m"(sw)
            : "m"(cmp)
            : "ax");
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"fcom_st_fstsw", bench_fcom_st_fstsw},
        {"fcom_m64_fstsw", bench_fcom_m64_fstsw},
        {"fcomp_st_fstsw", bench_fcomp_st_fstsw},
        {"fcomp_m64_fstsw", bench_fcomp_m64_fstsw},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        bench_ns_t sum = 0;
        for (int r = 0; r < RUNS; r++)
            sum += benches[i].fn();
        printf("BENCH %s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
