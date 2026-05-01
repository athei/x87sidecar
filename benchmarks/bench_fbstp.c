/*
 * bench_fbstp.c — FBSTP m80bcd throughput.
 *
 * Three patterns:
 *   small     — small magnitude (fits in low bytes; the divmod chain still
 *               runs all 18 digit pairs but most produce zero)
 *   large     — full-range value within 2^53 (16-digit BCD)
 *   negative  — same magnitude with sign set (exercises sign extraction)
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 2000000
#define RUNS 5

static bench_ns_t bench_fbstp_small(void) {
    volatile double in = 42.0;
    volatile uint8_t out[10];
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fldl  %1\n\t"
            "fbstp %0\n"
            : "=m"(out)
            : "m"(in)
            : "st");
    }
    return bench_now_ns() - start;
}

static bench_ns_t bench_fbstp_large(void) {
    volatile double in = 1234567890123456.0;
    volatile uint8_t out[10];
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fldl  %1\n\t"
            "fbstp %0\n"
            : "=m"(out)
            : "m"(in)
            : "st");
    }
    return bench_now_ns() - start;
}

static bench_ns_t bench_fbstp_negative(void) {
    volatile double in = -9007199254740991.0;
    volatile uint8_t out[10];
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fldl  %1\n\t"
            "fbstp %0\n"
            : "=m"(out)
            : "m"(in)
            : "st");
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"fbstp_small", bench_fbstp_small},
        {"fbstp_large", bench_fbstp_large},
        {"fbstp_negative", bench_fbstp_negative},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        bench_ns_t sum = 0;
        for (int r = 0; r < RUNS; r++)
            sum += benches[i].fn();
        printf("BENCH %-15s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
