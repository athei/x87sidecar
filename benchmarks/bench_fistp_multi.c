/*
 * bench_fistp_multi.c -- Benchmarks for RC caching with multiple FIST/FISTP
 * in a single IR run.
 *
 * Uses FIST (non-popping) to maximize store-to-setup ratio: one FLD feeds
 * N stores.  Stores to different array slots to avoid store-buffer contention.
 * Last store uses FISTP (popping) to clean up the x87 stack.
 *
 * The per-store cost should decrease for higher N as the control_word
 * LDRH+UBFX is hoisted once and amortized across all RC dispatches.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 10000000
#define RUNS 5

/* ── FLD + 1 FISTP (baseline: 1 store, no RC caching) ────────────────────── */

static bench_ns_t bench_fistp_x1(void) {
    volatile int32_t r[2];
    volatile double src = 42.7;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl  %1\n\t"
            "fistpl %0\n"
            : "=m"(r[0])
            : "m"(src));
    return bench_now_ns() - start;
}

/* ── FLD + 1 FIST + 1 FISTP (2 stores, RC caching activates) ─────────────── */

static bench_ns_t bench_fist_x2(void) {
    volatile int32_t r[2];
    volatile double src = 42.7;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl  %2\n\t"
            "fistl  %0\n\t"
            "fistpl %1\n"
            : "=m"(r[0]), "=m"(r[1])
            : "m"(src));
    return bench_now_ns() - start;
}

/* ── FLD + 3 FIST + 1 FISTP (4 stores) ───────────────────────────────────── */

static bench_ns_t bench_fist_x4(void) {
    volatile int32_t r[4];
    volatile double src = 42.7;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl  %4\n\t"
            "fistl  %0\n\t"
            "fistl  %1\n\t"
            "fistl  %2\n\t"
            "fistpl %3\n"
            : "=m"(r[0]), "=m"(r[1]), "=m"(r[2]), "=m"(r[3])
            : "m"(src));
    return bench_now_ns() - start;
}

/* ── FLD + 7 FIST + 1 FISTP (8 stores) ───────────────────────────────────── */

static bench_ns_t bench_fist_x8(void) {
    volatile int32_t r[8];
    volatile double src = 42.7;
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(
            "fldl  %8\n\t"
            "fistl  %0\n\t"
            "fistl  %1\n\t"
            "fistl  %2\n\t"
            "fistl  %3\n\t"
            "fistl  %4\n\t"
            "fistl  %5\n\t"
            "fistl  %6\n\t"
            "fistpl %7\n"
            : "=m"(r[0]), "=m"(r[1]), "=m"(r[2]), "=m"(r[3]), "=m"(r[4]), "=m"(r[5]), "=m"(r[6]),
              "=m"(r[7])
            : "m"(src));
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"fistp_x1", bench_fistp_x1},
        {"fist_x2", bench_fist_x2},
        {"fist_x4", bench_fist_x4},
        {"fist_x8", bench_fist_x8},
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
