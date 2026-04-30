/*
 * test_fstenv.c — Tests for FSTENV / FNSTENV m28byte (D9 /6 / 9B D9 /6).
 *
 * 32-bit protected-mode FPU env layout (28 bytes):
 *   +0  control_word (16b, padded to 32)
 *   +4  status_word  (16b)  ← TOP at bits [13:11]
 *   +8  tag_word     (16b)
 *   +12 FIP (4)  +16 FCS (2) +18 FOP (2) +20 FDP (4) +24 FDS (2) +26 reserved (2)
 *
 * Our JIT does not track FIP/FCS/FOP/FDP/FDS — those bytes are written
 * as zero.  Native hardware writes the actual last-instruction pointers
 * there, so tests that compare those bytes between native and JIT will
 * differ.  Tests below only assert CW/SW/TW correctness.
 *
 * Verification: snapshot via FSTENV, then mutate state and snapshot again,
 * comparing the relevant 6 bytes (CW, SW, TW).
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void fnstenv28(uint8_t *env28) {
    /* Use FNSTENV (no WAIT prefix) for deterministic encoding. */
    __asm__ volatile (".byte 0x9B");                      /* WAIT (innocuous) */
    __asm__ volatile ("fnstenv %0" : "=m"(*env28) :: "memory");
}

static uint16_t env_cw(const uint8_t *e) { return (uint16_t)e[0] | ((uint16_t)e[1] << 8); }
static uint16_t env_sw(const uint8_t *e) { return (uint16_t)e[4] | ((uint16_t)e[5] << 8); }
static uint16_t env_tw(const uint8_t *e) { return (uint16_t)e[8] | ((uint16_t)e[9] << 8); }

static void check_eq(const char *name, uint16_t got, uint16_t expected) {
    if (got == expected) {
        printf("PASS  %s  (0x%04x)\n", name, got);
    } else {
        printf("FAIL  %s  got=0x%04x expected=0x%04x\n", name, got, expected);
        failures++;
    }
}

/* ── Test 1: post-fninit, fstenv reads default state ─────────────────────── */
static void test_default(void) {
    __asm__ volatile (".byte 0xDB, 0xE3");  /* FNINIT */
    uint8_t env[28] __attribute__((aligned(16))) = {0};
    fnstenv28(env);
    check_eq("test_default: CW", env_cw(env), 0x037F);
    check_eq("test_default: SW", env_sw(env), 0x0000);
    check_eq("test_default: TW", env_tw(env), 0xFFFF);
}

/* ── Test 2: after pushing values, TOP and TW reflect the push ────────────── */
static void test_after_push(void) {
    __asm__ volatile (".byte 0xDB, 0xE3");
    /* Push 3 values: TOP becomes (0-1-1-1) & 7 = 5; slots 5/6/7 marked valid. */
    __asm__ volatile ("fld1");
    __asm__ volatile ("fld1");
    __asm__ volatile ("fld1");

    uint8_t env[28] __attribute__((aligned(16))) = {0};
    fnstenv28(env);
    uint16_t top = (env_sw(env) >> 11) & 7;
    check_eq("test_after_push: TOP=5", top, 5);
    /* TW: slots 5,6,7 valid (00 each), slots 0..4 empty (11 each).
     * Expected = 0b00_00_00_11_11_11_11_11 = 0x03FF. */
    check_eq("test_after_push: TW", env_tw(env), 0x03FF);

    /* Clean up the stack. */
    __asm__ volatile ("fstp %%st(0)" : : : "st");
    __asm__ volatile ("fstp %%st(0)" : : : "st");
    __asm__ volatile ("fstp %%st(0)" : : : "st");
}

/* ── Test 3: CW changes via fldcw are visible via fstenv ─────────────────── */
static void test_cw_change(void) {
    __asm__ volatile (".byte 0xDB, 0xE3");

    uint16_t cw_alt = 0x0F7F; /* round-toward-zero, all masked */
    __asm__ volatile ("fldcw %0" : : "m"(cw_alt));

    uint8_t env[28] __attribute__((aligned(16))) = {0};
    fnstenv28(env);
    check_eq("test_cw_change: CW", env_cw(env), 0x0F7F);

    /* Restore default. */
    uint16_t cw_def = 0x037F;
    __asm__ volatile ("fldcw %0" : : "m"(cw_def));
}

/* ── Test 4: deferred top/tag flush — push then fstenv reflects the push ── */
static void test_deferred_flush(void) {
    /* Re-init then push so the cache may have a deferred top/tag pending
     * at fstenv time.  fstenv must flush them so SW.TOP and TW are coherent. */
    __asm__ volatile (".byte 0xDB, 0xE3");
    __asm__ volatile ("fld1");
    __asm__ volatile ("fld1");

    uint8_t env[28] __attribute__((aligned(16))) = {0};
    fnstenv28(env);
    uint16_t top = (env_sw(env) >> 11) & 7;
    check_eq("test_deferred_flush: TOP=6", top, 6);
    /* TW: slots 6, 7 valid; rest empty. 0b00_00_00_00_11_11_11_11_11_11 */
    /* That's 0x0FFF. */
    check_eq("test_deferred_flush: TW", env_tw(env), 0x0FFF);

    __asm__ volatile ("fstp %%st(0)" : : : "st");
    __asm__ volatile ("fstp %%st(0)" : : : "st");
}

int main(void) {
    test_default();
    test_after_push();
    test_cw_change();
    test_deferred_flush();

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
