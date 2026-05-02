/*
 * test_fst_chain_compose.c — bit-exact validation of the StoreF32-run
 * coalescer's three lowering paths (STR Q × 4-floats, STP S, S × 2-floats,
 * scalar STR S fallback).
 *
 * Each test sets ST(0) to a known double, emits an N=2/3/4/12 chain of
 * `fsts` to a buffer, then pops ST(0) and compares every slot against
 * `(float)((double)src)` bit-exactly.  The buffer is arranged so that
 * the lowering picks the path we want to exercise:
 *
 *   - aligned-contiguous   → STR Q (N=4 → one quad; N=12 → three quads)
 *   - unaligned-contiguous → STP S, S (no STR Q because base+8 isn't 16-aligned)
 *   - scattered (stride 32)→ scalar STR S × N (no merge applies)
 *
 * If the coalescer mis-encodes any of the new helpers (DUP / STR Q / STP S)
 * a slot will mismatch and the test reports it.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static uint32_t as_u32(float f) {
    uint32_t u;
    memcpy(&u, &f, 4);
    return u;
}

static void check_slot(const char* name, int idx, float got, float expected) {
    if (as_u32(got) != as_u32(expected)) {
        printf("FAIL  %-50s slot=%-2d got=0x%08x expected=0x%08x\n", name, idx, as_u32(got),
               as_u32(expected));
        failures++;
    }
}

static void mark_pass(const char* name) {
    printf("PASS  %s\n", name);
}

/* ---- aligned-contiguous: 12 stores at offsets 0,4,...,44 (16-aligned base) */
static float aligned_buf[12] __attribute__((aligned(16)));

static void run_aligned_n12(double src) {
    memset(aligned_buf, 0, sizeof(aligned_buf));
    __asm__ volatile(
        "fldl %1\n\t"
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
        : "r"(aligned_buf), "m"(src)
        : "memory");
}

/* N=4 aligned: hits STR Q exactly once (one quad). */
static void run_aligned_n4(double src, float* out) {
    memset(out, 0, sizeof(float) * 4);
    __asm__ volatile(
        "fldl %1\n\t"
        "fsts (%0)\n\t"
        "fsts 4(%0)\n\t"
        "fsts 8(%0)\n\t"
        "fstps 12(%0)\n"
        :
        : "r"(out), "m"(src)
        : "memory");
}

/* N=3 aligned: STP S × 1 + STR S × 1 (no quad — only 3 stores). */
static void run_aligned_n3(double src, float* out) {
    memset(out, 0, sizeof(float) * 3);
    __asm__ volatile(
        "fldl %1\n\t"
        "fsts (%0)\n\t"
        "fsts 4(%0)\n\t"
        "fstps 8(%0)\n"
        :
        : "r"(out), "m"(src)
        : "memory");
}

/* N=2 aligned: STP S × 1. */
static void run_aligned_n2(double src, float* out) {
    memset(out, 0, sizeof(float) * 2);
    __asm__ volatile(
        "fldl %1\n\t"
        "fsts (%0)\n\t"
        "fstps 4(%0)\n"
        :
        : "r"(out), "m"(src)
        : "memory");
}

/* ---- unaligned-contiguous: base+8 (8-aligned, not 16-aligned) */
static float unaligned_buf[16] __attribute__((aligned(16)));

static void run_unaligned_n12(double src) {
    memset(unaligned_buf, 0, sizeof(unaligned_buf));
    __asm__ volatile(
        "fldl %1\n\t"
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
        : "r"(unaligned_buf), "m"(src)
        : "memory");
}

/* ---- scattered: stride 32 (forces scalar STR S × N) */
static float scattered_buf[96] __attribute__((aligned(16)));

static void run_scattered_n12(double src) {
    memset(scattered_buf, 0, sizeof(scattered_buf));
    __asm__ volatile(
        "fldl %1\n\t"
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
        : "r"(scattered_buf), "m"(src)
        : "memory");
}

static void test_value(double src, const char* tag) {
    const float expected = (float)src;
    char name[80];
    float small[4];

    /* aligned chains */
    snprintf(name, sizeof(name), "aligned_n2 (%s)", tag);
    run_aligned_n2(src, small);
    for (int i = 0; i < 2; i++)
        check_slot(name, i, small[i], expected);
    if (failures == 0)
        mark_pass(name);

    snprintf(name, sizeof(name), "aligned_n3 (%s)", tag);
    int prev = failures;
    run_aligned_n3(src, small);
    for (int i = 0; i < 3; i++)
        check_slot(name, i, small[i], expected);
    if (failures == prev)
        mark_pass(name);

    snprintf(name, sizeof(name), "aligned_n4 (%s)", tag);
    prev = failures;
    run_aligned_n4(src, small);
    for (int i = 0; i < 4; i++)
        check_slot(name, i, small[i], expected);
    if (failures == prev)
        mark_pass(name);

    snprintf(name, sizeof(name), "aligned_n12 (%s)", tag);
    prev = failures;
    run_aligned_n12(src);
    for (int i = 0; i < 12; i++)
        check_slot(name, i, aligned_buf[i], expected);
    if (failures == prev)
        mark_pass(name);

    /* unaligned */
    snprintf(name, sizeof(name), "unaligned_n12 (%s)", tag);
    prev = failures;
    run_unaligned_n12(src);
    for (int i = 2; i < 14; i++) /* offsets 8..52 / 4 = slots 2..13 */
        check_slot(name, i, unaligned_buf[i], expected);
    if (failures == prev)
        mark_pass(name);

    /* scattered: stride 32 bytes = stride 8 in float[] */
    snprintf(name, sizeof(name), "scattered_n12 (%s)", tag);
    prev = failures;
    run_scattered_n12(src);
    for (int slot = 0; slot < 12; slot++)
        check_slot(name, slot, scattered_buf[slot * 8], expected);
    if (failures == prev)
        mark_pass(name);
}

int main(void) {
    test_value(1.0, "1.0");
    test_value(-3.14159265358979, "-pi");
    test_value(2.7182818284590452, "e");
    test_value(0.0, "0.0");
    test_value(1e30, "1e30 (overflows to inf in f32)");
    test_value(1e-40, "1e-40 (denormal in f32)");
    test_value(1.0 / 0.0, "+inf");
    test_value(0.0 / 0.0, "nan");

    if (failures == 0) {
        printf("\nALL PASS\n");
        return 0;
    }
    printf("\n%d FAIL(s)\n", failures);
    return 1;
}
