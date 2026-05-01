/*
 * test_fldenv_compose.c — adversarial composition test for FLDENV.
 *
 * Guards against an m108-style internal-offset bug.  Pattern:
 *   1. Build a 28-byte env buffer with distinctive CW/SW/TW values.
 *   2. Run FLDENV (fall-through to stock if not in is_handled_x87).
 *   3. Read CW/SW/TW back via inline ops (FNSTCW/FNSTSW/FNSTENV).
 *   4. Assert exact match against what we put in the buffer.
 *
 * If stock fldenv wrote SW at offset 4 (m108-padded) of our X87State
 * instead of offset 2, our FNSTSW reading offset 2 returns 0 (stale
 * post-fninit) instead of the loaded value → test FAILS.
 *
 * Distinctive bit patterns chosen so a 1-byte-offset misread gives a
 * clearly different value.
 */
#include <stdint.h>
#include <stdio.h>

static int failures = 0;

static uint16_t fnstcw(void) {
    uint16_t cw;
    __asm__ volatile("fnstcw %0" : "=m"(cw));
    return cw;
}

static uint16_t fnstsw(void) {
    uint16_t sw;
    __asm__ volatile("fnstsw %0" : "=m"(sw));
    return sw;
}

static uint16_t fnstenv_tw(void) {
    uint8_t env[28] __attribute__((aligned(16)));
    __asm__ volatile("fnstenv %0" : "=m"(env)::"memory");
    return (uint16_t)env[8] | ((uint16_t)env[9] << 8);
}

static void check_eq_u16(const char* name, uint16_t got, uint16_t expected) {
    if (got == expected) {
        printf("PASS  %s  (got=0x%04x)\n", name, got);
    } else {
        printf("FAIL  %s  got=0x%04x expected=0x%04x\n", name, got, expected);
        failures++;
    }
}

/* ── Adversarial test: build env with distinctive values, fldenv,
 * verify each field round-trips through the load. */
static void test_fldenv_round_trip(void) {
    __asm__ volatile("fninit");

    /* Build env buffer.  m28byte layout (32-bit protected mode):
     *   [0..1]  CW   (16-bit, padded to 32)
     *   [4..5]  SW   (16-bit)
     *   [8..9]  TW   (16-bit)
     *   [12..27] FIP/FCS/FOP/FDP/FDS — ignored by spec on fldenv. */
    uint8_t env[28] __attribute__((aligned(16))) = {0};

    /* CW: 0x0F7F — precision=64-bit, round-to-nearest, all exceptions
     * masked.  Distinctive vs default 0x037F because bits 8,9,10
     * differ. */
    env[0] = 0x7F;
    env[1] = 0x0F;

    /* SW: 0x2800 — TOP=5 (bits 13:11 = 101), no condition codes,
     * no exceptions.  The TOP=5 value is the key to detect SW-offset
     * bugs because TOP lives in the high byte of SW. */
    env[4] = 0x00;
    env[5] = 0x28;

    /* TW: 0xCCCC — alternating valid/empty per slot pair.
     * Distinctive vs default 0xFFFF (all empty) and 0x0000 (all valid). */
    env[8] = 0xCC;
    env[9] = 0xCC;

    /* Cross-path: fldenv may fall through to stock. */
    __asm__ volatile("fldenv %0" : : "m"(env) : "memory");

    /* Verify each field via inline read. */
    check_eq_u16("test_fldenv_round_trip: CW=0x0F7F", fnstcw(), 0x0F7F);

    uint16_t sw_after = fnstsw();
    /* Not all 16 bits of SW are user-writable via fldenv (some bits
     * are sticky exception flags), but TOP and the basic CC bits are.
     * Check TOP=5 specifically. */
    check_eq_u16("test_fldenv_round_trip: TOP=5", (sw_after >> 11) & 7, 5);

    check_eq_u16("test_fldenv_round_trip: TW=0xCCCC", fnstenv_tw(), 0xCCCC);
}

/* ── Verify a follow-up FLD honours the loaded TOP — proves the cache
 * picked up the new TOP correctly after the cross-path op. */
static void test_fldenv_fld_after(void) {
    __asm__ volatile("fninit");
    /* Build env with TOP=5, all slots tagged empty (so the FLD push
     * succeeds without overflow). */
    uint8_t env[28] __attribute__((aligned(16))) = {0};
    env[0] = 0x7F;
    env[1] = 0x03; /* CW=0x037F default */
    env[4] = 0x00;
    env[5] = 0x28; /* SW: TOP=5 */
    env[8] = 0xFF;
    env[9] = 0xFF; /* TW: all empty */
    __asm__ volatile("fldenv %0" : : "m"(env) : "memory");

    uint16_t top_pre = (fnstsw() >> 11) & 7;
    check_eq_u16("test_fldenv_fld_after: pre-FLD TOP=5", top_pre, 5);

    /* FLD pre-decrements TOP, so 5 → 4. */
    __asm__ volatile("fld1");

    uint16_t top_post = (fnstsw() >> 11) & 7;
    check_eq_u16("test_fldenv_fld_after: post-FLD TOP=4", top_post, 4);

    /* Verify the value. */
    double x = 0.0;
    __asm__ volatile("fstpl %0" : "=m"(x));
    if (x != 1.0) {
        printf("FAIL  test_fldenv_fld_after: ST(0) — got=%g expected=1.0\n", x);
        failures++;
    } else {
        printf("PASS  test_fldenv_fld_after: ST(0)=1.0\n");
    }
}

int main(void) {
    test_fldenv_round_trip();
    test_fldenv_fld_after();

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
