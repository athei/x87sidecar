/*
 * bench_fldenv.c — FLDENV throughput.
 *
 * Loop loads the same 28-byte env buffer every iteration.  3 LDRH+STRH
 * pairs + 1 UBFX = 7 instructions in the JIT body.
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#define TIMES 2000000
#define RUNS  5

static clock_t bench_fldenv_loop(void) {
    /* Build a self-consistent env once. */
    uint8_t env[28] __attribute__((aligned(16))) = {0};
    __asm__ volatile (".byte 0xDB, 0xE3");           /* FNINIT */
    __asm__ volatile ("fnstenv %0" : "=m"(env) :: "memory");
    /* Set TOP=3 in SW. */
    env[5] = (env[5] & ~0x38) | (3 << 3);

    clock_t start = clock();
    for (int i = 0; i < TIMES; i++) {
        __asm__ volatile ("fldenv %0" : : "m"(env) : "memory");
    }
    return clock() - start;
}

int main(void) {
    struct { const char *name; clock_t (*fn)(void); } benches[] = {
        {"fldenv_loop", bench_fldenv_loop},
    };
    int n = (int)(sizeof(benches) / sizeof(benches[0]));
    for (int i = 0; i < n; i++) {
        clock_t sum = 0;
        for (int r = 0; r < RUNS; r++) sum += benches[i].fn();
        printf("BENCH %-15s %lu\n", benches[i].name, (unsigned long)(sum / RUNS));
    }
    return 0;
}
