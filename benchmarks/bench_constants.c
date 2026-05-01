/*
 * bench_constants.c -- Benchmarks for x87 constant-loading opcodes.
 * Covers: FLDZ, FLD1, FLDL2E, FLDL2T, FLDLG2, FLDLN2, FLDPI
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 1000000
#define RUNS 5

static bench_ns_t bench_fldz(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile("fldz\n\t fstpl %0\n" : "=m"(r));
    return bench_now_ns() - start;
}

static bench_ns_t bench_fld1(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile("fld1\n\t fstpl %0\n" : "=m"(r));
    return bench_now_ns() - start;
}

static bench_ns_t bench_fldl2e(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile("fldl2e\n\t fstpl %0\n" : "=m"(r));
    return bench_now_ns() - start;
}

static bench_ns_t bench_fldl2t(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile("fldl2t\n\t fstpl %0\n" : "=m"(r));
    return bench_now_ns() - start;
}

static bench_ns_t bench_fldlg2(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile("fldlg2\n\t fstpl %0\n" : "=m"(r));
    return bench_now_ns() - start;
}

static bench_ns_t bench_fldln2(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile("fldln2\n\t fstpl %0\n" : "=m"(r));
    return bench_now_ns() - start;
}

static bench_ns_t bench_fldpi(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile("fldpi\n\t fstpl %0\n" : "=m"(r));
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"fldz", bench_fldz},     {"fld1", bench_fld1},     {"fldl2e", bench_fldl2e},
        {"fldl2t", bench_fldl2t}, {"fldlg2", bench_fldlg2}, {"fldln2", bench_fldln2},
        {"fldpi", bench_fldpi},
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
