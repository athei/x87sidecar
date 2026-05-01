/*
 * bench_add.c -- Benchmarks for x87 addition opcodes.
 * Covers: FADD (m64, ST(0)/ST(i)), FADDP, FIADD (m32)
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 1000000
#define RUNS 5

static bench_ns_t bench_fadd_m64(void) {
    bench_ns_t start = bench_now_ns();
    volatile double mem = 2.5;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"
            "faddl %1\n\t"
            "fstpl %0\n"
            : "=m"(r)
            : "m"(mem));
    return bench_now_ns() - start;
}

static bench_ns_t bench_fadd_st(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"
            "fld1\n\t"
            "fadd %%st(1), %%st(0)\n\t"
            "fstpl %0\n\t"
            "fstp %%st(0)\n\t"
            : "=m"(r));
    return bench_now_ns() - start;
}

static bench_ns_t bench_faddp(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"
            "fld1\n\t"
            "faddp\n\t"
            "fstpl %0\n"
            : "=m"(r));
    return bench_now_ns() - start;
}

static bench_ns_t bench_fiadd_m32(void) {
    bench_ns_t start = bench_now_ns();
    volatile int32_t mem = 7;
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t"
            "fiaddl %1\n\t"
            "fstpl %0\n"
            : "=m"(r)
            : "m"(mem));
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"fadd_m64", bench_fadd_m64},
        {"fadd_st", bench_fadd_st},
        {"faddp", bench_faddp},
        {"fiadd_m32", bench_fiadd_m32},
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
