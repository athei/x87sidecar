/*
 * bench_single_op.c -- Benchmark for the single-op fast path
 * (TranslatorX87Single.cpp): isolated (run==1) fld / fst / fstp.
 *
 * Every x87 instruction is fenced by integer instructions INSIDE the asm
 * block so X87Cache::lookahead sees a run of exactly 1 — the ABI-bridge
 * shape (ST0 return / spill at call boundaries).  Neither the IR, the
 * peephole, nor the register cache can amortize these.
 *
 * Compare with X87_DISABLE_SINGLE_FAST=1 to measure the fast path's benefit
 * (same binary, translated through the generic uncached emitters instead).
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 2000000
#define RUNS 5

/* Integer fence: breaks the x87 run so each op translates in isolation. */
#define FENCE "xorl %%ecx, %%ecx\n\taddl $1, %%ecx\n\t"

/* --------------------------------------------------------------------------
 * Isolated fld m32 + isolated fstp m32: two single-op blocks per round trip.
 * This is the exact hook-boundary bridge shape (materialize ST0 / spill ST0).
 * -------------------------------------------------------------------------- */
static bench_ns_t bench_single_fld_fstp_m32(void) {
    bench_ns_t start = bench_now_ns();
    volatile float r;
    float a = 1.5f;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(FENCE
                         "flds %1\n\t"
                         FENCE
                         "fstps %0\n\t"
                         FENCE
                         : "=m"(r)
                         : "m"(a)
                         : "ecx");
    return bench_now_ns() - start;
}

/* --------------------------------------------------------------------------
 * Isolated fld m64 + isolated fstp m64.
 * -------------------------------------------------------------------------- */
static bench_ns_t bench_single_fld_fstp_m64(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    double a = 3.141592653589793;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(FENCE
                         "fldl %1\n\t"
                         FENCE
                         "fstpl %0\n\t"
                         FENCE
                         : "=m"(r)
                         : "m"(a)
                         : "ecx");
    return bench_now_ns() - start;
}

/* --------------------------------------------------------------------------
 * Isolated fld m64 + isolated fst m32 (non-pop) + isolated fstp m64:
 * three single-op blocks, covering the stateless fst path too.
 * -------------------------------------------------------------------------- */
static bench_ns_t bench_single_fld_fst_fstp(void) {
    bench_ns_t start = bench_now_ns();
    volatile float rf;
    volatile double rd;
    double a = 2.718281828459045;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(FENCE
                         "fldl %2\n\t"
                         FENCE
                         "fsts %0\n\t"
                         FENCE
                         "fstpl %1\n\t"
                         FENCE
                         : "=m"(rf), "=m"(rd)
                         : "m"(a)
                         : "ecx");
    return bench_now_ns() - start;
}

/* --------------------------------------------------------------------------
 * Width-bisection variants: isolate which op regresses.
 * fld m32 + fstp m64, and fld m64 + fstp m32.
 * -------------------------------------------------------------------------- */
static bench_ns_t bench_single_fld32_fstp64(void) {
    bench_ns_t start = bench_now_ns();
    volatile double r;
    float a = 1.5f;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(FENCE
                         "flds %1\n\t"
                         FENCE
                         "fstpl %0\n\t"
                         FENCE
                         : "=m"(r)
                         : "m"(a)
                         : "ecx");
    return bench_now_ns() - start;
}

static bench_ns_t bench_single_fld64_fstp32(void) {
    bench_ns_t start = bench_now_ns();
    volatile float r;
    double a = 3.141592653589793;
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(FENCE
                         "fldl %1\n\t"
                         FENCE
                         "fstps %0\n\t"
                         FENCE
                         : "=m"(r)
                         : "m"(a)
                         : "ecx");
    return bench_now_ns() - start;
}

/* --------------------------------------------------------------------------
 * Layout-controlled m32 variant: source and destination on separate cache
 * lines.  If the plain m32 pair's result diverges from this one, the delta
 * is a guest stack-layout store-to-load hazard (r adjacent to a), not a
 * property of the translated x87 code.
 * -------------------------------------------------------------------------- */
static struct {
    float a;
    char pad1[124];
    volatile float r;
    char pad2[124];
} g_m32 __attribute__((aligned(128))) = {.a = 1.5f};

static bench_ns_t bench_single_fld_fstp_m32_padded(void) {
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++)
        __asm__ volatile(FENCE
                         "flds %1\n\t"
                         FENCE
                         "fstps %0\n\t"
                         FENCE
                         : "=m"(g_m32.r)
                         : "m"(g_m32.a)
                         : "ecx");
    return bench_now_ns() - start;
}

typedef struct {
    const char* name;
    bench_ns_t (*fn)(void);
} Bench;

int main(void) {
    Bench benches[] = {
        {"fld_fstp_m32_singles", bench_single_fld_fstp_m32},
        {"fld_fstp_m64_singles", bench_single_fld_fstp_m64},
        {"fld_fst_fstp_singles", bench_single_fld_fst_fstp},
        {"fld32_fstp64_singles", bench_single_fld32_fstp64},
        {"fld64_fstp32_singles", bench_single_fld64_fstp32},
        {"fld_fstp_m32_padded", bench_single_fld_fstp_m32_padded},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        benches[i].fn(); /* warmup: discard, JIT translates on first call */
        bench_ns_t sum = 0;
        for (int r = 0; r < RUNS; r++)
            sum += benches[i].fn();
        printf("BENCH %-20s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
