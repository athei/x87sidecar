/*
 * bench_fst_chain.c — Benchmark for fst m32 broadcast chains.
 *
 * Models the dominant uncovered pattern in the 2026-05-02 WoW capture:
 * a single ST(0) value stored to N consecutive m32 slots.  N=12 is the
 * length the WoW profile shows firing 61.7 M times per session.
 *
 * Three address-shape variants exercise the three ARM64 store-form
 * categories the future StoreF32 coalescer (X87IRLower.cpp) will pick
 * between:
 *   - aligned-contiguous   → 16-byte aligned base, offsets 0..44 step 4
 *                            (will hit DUP V.4S + STR Q × 3 post-fusion)
 *   - unaligned-contiguous → 8-byte aligned base, offsets 0..44 step 4
 *                            (will hit STP S, S × 6 post-fusion)
 *   - scattered            → 32-byte stride per slot
 *                            (will stay on scalar STR S × 12)
 *
 * All three feed an N=12 chain of `fsts` (f32 stores) ending with
 * `fstps` to drain ST(0).  The compiler can't reorder via the
 * "memory" clobber + volatile asm.
 *
 * Run via the rosettax87 loader (per feedback_runtime_loader.md).
 * Compare three configs:
 *   - X87_DISABLE_HOOK=1     (stock Rosetta, no JIT hook)
 *   - X87_DISABLE_X87_IR=1   (peephole + single-op only)
 *   - default                (X87IR-served path)
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "bench_timing.h"

#define TIMES 1000000
#define RUNS 5

/* 16-byte aligned 12-element f32 buffer for the aligned-contiguous case. */
static float aligned_buf[12] __attribute__((aligned(16)));

/* 16 elements (64 bytes) so the unaligned variant can start at +8 and
 * cover offsets 8..52 without overrunning. */
static float unaligned_buf[16] __attribute__((aligned(16)));

/* Stride-32 scattered buffer.  12 slots × 32 bytes = 384 bytes total.
 * Each fst targets a distinct cache line within reason. */
static float scattered_buf[96] __attribute__((aligned(16)));

/* Aligned, contiguous: 12 stores at base+{0,4,...,44}, base 16-aligned. */
static bench_ns_t bench_fst_chain_aligned(void) {
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fld1\n\t"
            "fsts (%0)\n\t"
            "fsts 4(%0)\n\t"
            "fsts 8(%0)\n\t"
            "fsts 12(%0)\n\t"
            "fsts 16(%0)\n\t"
            "fsts 20(%0)\n\t"
            "fsts 24(%0)\n\t"
            "fsts 28(%0)\n\t"
            "fsts 32(%0)\n\t"
            "fsts 36(%0)\n\t"
            "fsts 40(%0)\n\t"
            "fstps 44(%0)\n"
            :
            : "r"(aligned_buf)
            : "memory");
    }
    return bench_now_ns() - start;
}

/* Unaligned, contiguous: 12 stores at base+{8,12,...,52}, base+8 is
 * 8-byte aligned but not 16-byte aligned — STR Q can't fire, but STP S
 * pairs can. */
static bench_ns_t bench_fst_chain_unaligned(void) {
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fld1\n\t"
            "fsts 8(%0)\n\t"
            "fsts 12(%0)\n\t"
            "fsts 16(%0)\n\t"
            "fsts 20(%0)\n\t"
            "fsts 24(%0)\n\t"
            "fsts 28(%0)\n\t"
            "fsts 32(%0)\n\t"
            "fsts 36(%0)\n\t"
            "fsts 40(%0)\n\t"
            "fsts 44(%0)\n\t"
            "fsts 48(%0)\n\t"
            "fstps 52(%0)\n"
            :
            : "r"(unaligned_buf)
            : "memory");
    }
    return bench_now_ns() - start;
}

/* Scattered: 12 stores with stride 32 — addresses are non-contiguous,
 * so neither STR Q nor STP S applies.  Pure STR S fallback. */
static bench_ns_t bench_fst_chain_scattered(void) {
    bench_ns_t start = bench_now_ns();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile(
            "fld1\n\t"
            "fsts 0(%0)\n\t"
            "fsts 32(%0)\n\t"
            "fsts 64(%0)\n\t"
            "fsts 96(%0)\n\t"
            "fsts 128(%0)\n\t"
            "fsts 160(%0)\n\t"
            "fsts 192(%0)\n\t"
            "fsts 224(%0)\n\t"
            "fsts 256(%0)\n\t"
            "fsts 288(%0)\n\t"
            "fsts 320(%0)\n\t"
            "fstps 352(%0)\n"
            :
            : "r"(scattered_buf)
            : "memory");
    }
    return bench_now_ns() - start;
}

int main(void) {
    struct {
        const char* name;
        bench_ns_t (*fn)(void);
    } benches[] = {
        {"fst_chain_aligned_n12", bench_fst_chain_aligned},
        {"fst_chain_unaligned_n12", bench_fst_chain_unaligned},
        {"fst_chain_scattered_n12", bench_fst_chain_scattered},
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
