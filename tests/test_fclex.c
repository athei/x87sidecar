/*
 * test_fclex.c — Tests for FCLEX / FNCLEX (DB E2, optionally prefixed by 9B).
 *
 * Spec: clears bits 0..7 (PE,UE,OE,ZE,DE,IE,SF,ES) and bit 15 (B) of the
 * x87 status_word.  Preserves C0,C1,C2 (bits 8..10), TOP (bits 11..13),
 * and C3 (bit 14).  In mask form: status_word &= 0x7F00.
 *
 * Bits 0..6 (PE,UE,OE,ZE,DE,IE,SF) are injected via fnstenv/fldenv
 * (modify the saved env in place, reload).  ES (bit 7) and B (bit 15)
 * cannot be set by fldenv in isolation — the FPU clamps them to 0 unless
 * an unmasked exception is actually pending — so we skip injecting them.
 * Their clearing is implied by the AND-mask design.
 *
 * Avoids depending on arithmetic exception propagation (our path doesn't
 * track ZE through fdiv-by-zero, so "1/0 sets ZE" would be unreliable).
 */
#include <stdint.h>
#include <stdio.h>

static int failures = 0;

static uint16_t fnstsw(void) {
    uint16_t sw;
    __asm__ volatile("fnstsw %0" : "=m"(sw));
    return sw;
}

/* 32-bit protected-mode FPU env (28 bytes):
 *   +0  control_word (16b, padded to 32)
 *   +4  status_word  (16b, padded to 32)
 *   +8  tag_word     (16b, padded to 32)
 *  ... (IP/operand pointers, not relevant here)
 */
static void inject_status_word(uint16_t or_bits) {
    uint8_t env[28] __attribute__((aligned(16)));
    __asm__ volatile("fnstenv %0" : "=m"(env)::"memory");
    uint16_t sw = (uint16_t)env[4] | ((uint16_t)env[5] << 8);
    sw |= or_bits;
    env[4] = sw & 0xFF;
    env[5] = sw >> 8;
    __asm__ volatile("fldenv %0" : : "m"(env) : "memory");
}

static void check_eq_u16(const char* name, uint16_t got, uint16_t expected) {
    if (got == expected) {
        printf("PASS  %s  (sw=0x%04x)\n", name, got);
    } else {
        printf("FAIL  %s  got=0x%04x expected=0x%04x\n", name, got, expected);
        failures++;
    }
}

static void check_zero_bits(const char* name, uint16_t got, uint16_t mask) {
    if ((got & mask) == 0) {
        printf("PASS  %s  (sw=0x%04x mask=0x%04x)\n", name, got, mask);
    } else {
        printf("FAIL  %s  sw=0x%04x mask=0x%04x — bits 0x%04x still set\n", name, got, mask,
               got & mask);
        failures++;
    }
}

static void check_set_bits(const char* name, uint16_t got, uint16_t mask) {
    if ((got & mask) == mask) {
        printf("PASS  %s  (sw=0x%04x bits=0x%04x)\n", name, got, got & mask);
    } else {
        printf("FAIL  %s  sw=0x%04x — expected bits 0x%04x set, got 0x%04x\n", name, got, mask,
               got & mask);
        failures++;
    }
}

/* ── Test 1: all-clearable bits set via fldenv → fnclex clears them. ──────── */
static void test_clears_all(void) {
    __asm__ volatile("fninit");
    inject_status_word(0x007F); /* bits 0..7 + B */

    uint16_t sw_before = fnstsw();
    check_set_bits("test_clears_all: bits set before fnclex", sw_before, 0x007F);

    __asm__ volatile("fnclex");

    uint16_t sw_after = fnstsw();
    check_zero_bits("test_clears_all: bits cleared after fnclex", sw_after, 0x007F);
}

/* ── Test 2: condition codes (C0,C1,C2,C3) survive fnclex ─────────────────── */
static void test_preserves_cc(void) {
    volatile double a = 0.0, b = 1.0;
    __asm__ volatile("fninit");

    /* fld 0.0 ; fcoml 1.0  → 0 < 1 → C3=0, C2=0, C0=1 (i.e. bit 8 set). */
    __asm__ volatile("fldl %0" : : "m"(a));
    __asm__ volatile("fcoml %0" : : "m"(b));

    /* Also inject the clearable bits so we test that fnclex distinguishes
     * C-bits from exception bits, not just "everything stays the same". */
    inject_status_word(0x007F);

    uint16_t sw_before = fnstsw();
    uint16_t cc_mask = 0x4700; /* C3 | C2 | C1 | C0 */
    uint16_t cc_before = sw_before & cc_mask;

    __asm__ volatile("fnclex");

    uint16_t sw_after = fnstsw();
    uint16_t cc_after = sw_after & cc_mask;

    check_eq_u16("test_preserves_cc: C-bits unchanged", cc_after, cc_before);
    /* Also verify the clearable bits did get cleared. */
    check_zero_bits("test_preserves_cc: clearable bits cleared", sw_after, 0x007F);

    __asm__ volatile("fstp %%st(0)" : : : "st");
}

/* ── Test 3: TOP field (bits 13:11) survives fnclex ───────────────────────── */
static void test_preserves_top(void) {
    __asm__ volatile("fninit");
    /* Default TOP=0 after fninit; fld1 decrements to 7. */
    __asm__ volatile("fld1");

    /* Inject clearable bits so fnclex actually has work to do. */
    inject_status_word(0x007F);

    uint16_t sw_before = fnstsw();
    uint16_t top_before = (sw_before >> 11) & 7;

    __asm__ volatile("fnclex");

    uint16_t sw_after = fnstsw();
    uint16_t top_after = (sw_after >> 11) & 7;

    if (top_before == 7 && top_after == 7) {
        printf("PASS  test_preserves_top  (top=%u before/after)\n", top_after);
    } else {
        printf("FAIL  test_preserves_top  top_before=%u top_after=%u\n", top_before, top_after);
        failures++;
    }
    /* And the clearable bits actually cleared. */
    check_zero_bits("test_preserves_top: clearable bits cleared", sw_after, 0x007F);

    __asm__ volatile("fstp %%st(0)" : : : "st");
}

/* ── Test 4: fnclex on a clean state is a no-op ───────────────────────────── */
static void test_clean_noop(void) {
    __asm__ volatile("fninit");

    uint16_t sw_before = fnstsw();
    __asm__ volatile("fnclex");
    uint16_t sw_after = fnstsw();

    check_eq_u16("test_clean_noop", sw_after, sw_before);
}

int main(void) {
    test_clears_all();
    test_preserves_cc();
    test_preserves_top();
    test_clean_noop();

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
