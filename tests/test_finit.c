/*
 * test_finit.c — Tests for FNINIT (DB E3) and FINIT (9B DB E3).
 *
 * Spec: reset the FPU.
 *   control_word ← 0x037F
 *   status_word  ← 0x0000   (TOP=0, all flags clear)
 *   tag_word     ← 0xFFFF   (all 8 slots tagged kEmpty)
 *
 * The 8 ST register slots are NOT cleared — tagged-empty suffices.
 *
 * We use FNINIT (no WAIT prefix) for deterministic encoding; clang's x86
 * assembler may optionally insert 9B in front, so encode as raw bytes.
 *
 * Verification path: fnstenv to a 28-byte buffer, read CW/SW/TW from
 * offsets 0/4/8.
 */
#include <stdint.h>
#include <stdio.h>

static int failures = 0;

static void fnstenv28(uint8_t* env28) {
    __asm__ volatile("fnstenv %0" : "=m"(*env28)::"memory");
}

static uint16_t env_cw(const uint8_t* e) {
    return (uint16_t)e[0] | ((uint16_t)e[1] << 8);
}
static uint16_t env_sw(const uint8_t* e) {
    return (uint16_t)e[4] | ((uint16_t)e[5] << 8);
}
static uint16_t env_tw(const uint8_t* e) {
    return (uint16_t)e[8] | ((uint16_t)e[9] << 8);
}

static void check_eq(const char* name, uint16_t got, uint16_t expected) {
    if (got == expected) {
        printf("PASS  %s  (0x%04x)\n", name, got);
    } else {
        printf("FAIL  %s  got=0x%04x expected=0x%04x\n", name, got, expected);
        failures++;
    }
}

/* ── Test 1: from a "dirty" state, fninit resets CW, SW, TW ───────────────── */
static void test_resets_from_dirty(void) {
    /* Build a dirty state: change CW, push some values to fill ST, set
     * exception bits via fldenv. */
    __asm__ volatile("fninit");

    /* Change CW to something non-default (round-toward-zero, mask all). */
    uint16_t cw_alt = 0x0F7F;
    __asm__ volatile("fldcw %0" : : "m"(cw_alt));

    /* Push 5 values so TOP = 8-5 = 3 and 5 tag-word slots are kValid. */
    __asm__ volatile("fld1");
    __asm__ volatile("fld1");
    __asm__ volatile("fld1");
    __asm__ volatile("fld1");
    __asm__ volatile("fld1");

    /* Inject exception bits into status_word. */
    {
        uint8_t env[28] __attribute__((aligned(16)));
        __asm__ volatile("fnstenv %0" : "=m"(env)::"memory");
        uint16_t sw = env_sw(env);
        sw |= 0x003F; /* bits 0..5 (PE,UE,OE,ZE,DE,IE) */
        env[4] = sw & 0xFF;
        env[5] = sw >> 8;
        __asm__ volatile("fldenv %0" : : "m"(env) : "memory");
    }

    /* Sanity: dirty state visible. */
    {
        uint8_t env[28] __attribute__((aligned(16)));
        fnstenv28(env);
        if (env_cw(env) != cw_alt) {
            printf("FAIL  dirty_setup_cw  got=0x%04x expected=0x%04x\n", env_cw(env), cw_alt);
            failures++;
        }
        /* Don't assert SW here — fldenv may clamp ES; just verify TOP shifted. */
        uint16_t top_dirty = (env_sw(env) >> 11) & 7;
        if (top_dirty != 3) {
            printf("FAIL  dirty_setup_top  got=%u expected=3\n", top_dirty);
            failures++;
        }
        if (env_tw(env) == 0xFFFF) {
            printf("FAIL  dirty_setup_tw  got=0xFFFF (expected non-empty slots)\n");
            failures++;
        }
    }

    /* The reset under test. */
    __asm__ volatile(".byte 0xDB, 0xE3"); /* FNINIT */

    /* Verify post-reset env. */
    uint8_t env[28] __attribute__((aligned(16)));
    fnstenv28(env);
    check_eq("test_resets_from_dirty: CW", env_cw(env), 0x037F);
    check_eq("test_resets_from_dirty: SW", env_sw(env), 0x0000);
    check_eq("test_resets_from_dirty: TW", env_tw(env), 0xFFFF);
}

/* ── Test 2: post-finit, fld1 pushes correctly (TOP=0 → 7) ───────────────── */
static void test_fld_after_finit(void) {
    __asm__ volatile(".byte 0xDB, 0xE3");

    /* TOP should be 0; fld1 decrements TOP to 7 and tags slot 7 valid. */
    __asm__ volatile("fld1");

    uint8_t env[28] __attribute__((aligned(16)));
    fnstenv28(env);
    uint16_t top = (env_sw(env) >> 11) & 7;
    uint16_t tw = env_tw(env);
    /* Tag for ST(0) (= phys slot 7 here) should be kValid (00). All other
     * slots stay kEmpty (11). So tw == 0x3FFF. */
    if (top == 7 && tw == 0x3FFF) {
        printf("PASS  test_fld_after_finit  top=%u tw=0x%04x\n", top, tw);
    } else {
        printf("FAIL  test_fld_after_finit  top=%u tw=0x%04x (expected top=7 tw=0x3FFF)\n", top,
               tw);
        failures++;
    }

    __asm__ volatile("fstp %%st(0)" : : : "st");
}

/* ── Test 3: fninit on a clean (already-init) state stays clean ──────────── */
static void test_idempotent(void) {
    __asm__ volatile(".byte 0xDB, 0xE3");
    __asm__ volatile(".byte 0xDB, 0xE3");

    uint8_t env[28] __attribute__((aligned(16)));
    fnstenv28(env);
    check_eq("test_idempotent: CW", env_cw(env), 0x037F);
    check_eq("test_idempotent: SW", env_sw(env), 0x0000);
    check_eq("test_idempotent: TW", env_tw(env), 0xFFFF);
}

int main(void) {
    test_resets_from_dirty();
    test_fld_after_finit();
    test_idempotent();

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
