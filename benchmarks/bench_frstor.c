/*
 * bench_frstor.c — FRSTOR throughput.
 *
 * Loop loads the same 108-byte FPU state every iteration.  Body emits
 * 1 ADD + 1 LDR + 1 LDRH + ~26 conversion insns + 5-insn store per slot
 * (8 slots) plus the env-header writes — total ~250 insns.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 2000000
#define RUNS 5

static void encode_f80(uint8_t* out10, double d) {
    long double ld = (long double)d;
    memcpy(out10, &ld, 10);
}

static void put16(uint8_t* p, uint16_t v) {
    p[0] = v & 0xFF;
    p[1] = v >> 8;
}

static bench_ns_t bench_frstor_loop(void) {
    /* Build a self-consistent 108-byte buffer once. */
    uint8_t buf[108] __attribute__((aligned(16))) = {0};
    put16(buf + 0, 0x037F);
    put16(buf + 4, 0x0000); /* TOP=0 */
    put16(buf + 8, 0x0000); /* all 8 slots valid */
    static const double vals[8] = {1.0, 2.5, -3.5, 1e10, -1e-10, 42.0, -7.5, 0.5};
    for (int i = 0; i < 8; ++i)
        encode_f80(buf + 0x1C + i * 10, vals[i]);

    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile("frstor %0" : : "m"(buf) : "memory");
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"frstor_loop", bench_frstor_loop},
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
