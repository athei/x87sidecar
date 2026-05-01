/*
 * bench_fistt.c -- Benchmarks for FISTT (FISTTP) direct translation.
 * Covers: FISTT m16, FISTT m32, FISTT m64, plus FISTP m32 for comparison.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 1000000
#define RUNS 5

static bench_ns_t bench_fistt_m16(void) {
    bench_ns_t start = bench_now_ns();
    volatile int16_t r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t fld1\n\t faddp\n\t" /* ST(0)=2 */
            "fisttps %0\n"
            : "=m"(r));
    return bench_now_ns() - start;
}

static bench_ns_t bench_fistt_m32(void) {
    bench_ns_t start = bench_now_ns();
    volatile int32_t r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t fld1\n\t faddp\n\t fld1\n\t faddp\n\t" /* ST(0)=3 */
            "fisttpl %0\n"
            : "=m"(r));
    return bench_now_ns() - start;
}

static bench_ns_t bench_fistt_m64(void) {
    bench_ns_t start = bench_now_ns();
    volatile int64_t r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t fld1\n\t faddp\n\t fld1\n\t faddp\n\t" /* ST(0)=3 */
            "fisttpll %0\n"
            : "=m"(r));
    return bench_now_ns() - start;
}

/* FISTP m32 for comparison — same workload but with RC dispatch overhead */
static bench_ns_t bench_fistp_m32(void) {
    bench_ns_t start = bench_now_ns();
    volatile int32_t r;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fld1\n\t fld1\n\t faddp\n\t fld1\n\t faddp\n\t" /* ST(0)=3 */
            "fistpl %0\n"
            : "=m"(r));
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"fistt_m16", bench_fistt_m16},
        {"fistt_m32", bench_fistt_m32},
        {"fistt_m64", bench_fistt_m64},
        {"fistp_m32", bench_fistp_m32},
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
