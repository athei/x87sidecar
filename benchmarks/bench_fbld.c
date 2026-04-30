/*
 * bench_fbld.c — FBLD m80bcd throughput.
 *
 * Three patterns:
 *   small    — single byte, 1-2 digit BCD (e.g., 42)
 *   medium   — 6-byte BCD with 12 nonzero digits (exercises full nibble unpack)
 *   negative — sign byte set, full 18-digit BCD (worst case, MADD chain + negate)
 *
 * Each iteration: FBLD m80 + FSTPL m64 (so the loop has to actually pop ST(0)
 * back to memory, preventing the optimizer from folding the chain).
 */
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "bench_timing.h"

#define TIMES 2000000
#define RUNS  5

static bench_ns_t bench_fbld_small(void) {
    volatile uint8_t bcd[10] = {0x42, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    volatile double r;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile (
            "fbld  %1\n\t"
            "fstpl %0\n"
            : "=m"(r) : "m"(bcd));
    }
    return bench_now_ns() - start;
}

static bench_ns_t bench_fbld_medium(void) {
    /* +123456789012  (12 nonzero digits, low 6 bytes) */
    volatile uint8_t bcd[10] = {0x12, 0x90, 0x78, 0x56, 0x34, 0x12, 0, 0, 0, 0};
    volatile double r;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile (
            "fbld  %1\n\t"
            "fstpl %0\n"
            : "=m"(r) : "m"(bcd));
    }
    return bench_now_ns() - start;
}

static bench_ns_t bench_fbld_negative(void) {
    /* -999999999999999999  (worst case: all 18 digits = 9, negate) */
    volatile uint8_t bcd[10] = {0x99,0x99,0x99,0x99,0x99,0x99,0x99,0x99,0x99,0x80};
    volatile double r;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile (
            "fbld  %1\n\t"
            "fstpl %0\n"
            : "=m"(r) : "m"(bcd));
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct { const char *name; bench_ns_t (*fn)(void); } benches[] = {
        {"fbld_small",    bench_fbld_small},
        {"fbld_medium",   bench_fbld_medium},
        {"fbld_negative", bench_fbld_negative},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        bench_ns_t sum = 0;
        for (int r = 0; r < RUNS; r++) sum += benches[i].fn();
        printf("BENCH %-15s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
