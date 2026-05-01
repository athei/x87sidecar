/*
 * test_finit_compose.c — adversarial composition test for FINIT/FNINIT.
 *
 * Guards against an m108-style internal-offset bug.  Pattern:
 *   1. Dirty CW/SW/TW with distinctive values via inline ops.
 *   2. Run FNINIT (fall-through to stock if not in is_handled_x87).
 *   3. Read CW/SW/TW back via inline ops (FNSTCW for CW, FNSTSW for SW,
 *      FNSTENV for TW).
 *   4. Assert reset-to-default values: CW=0x037F, SW=0, TW=0xFFFF.
 *
 * If stock wrote CW at offset 4 (m108-padded) instead of offset 0, our
 * FNSTCW reading offset 0 would still see the distinctive 0xC321 → fails.
 * Same for SW (offset 2 vs 4) and TW (offset 4 vs 8).
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

/* ── Adversarial test: dirty all three metadata words with distinctive
 * patterns, then FNINIT, then verify reset-to-default. */
static void test_finit_resets_all(void) {
    /* Setup: dirty CW with 0x0F7F (precision/round bits set distinctively).
     * Push 5 values so SW.TOP=3 and TW shows 5 valid + 3 empty. */
    uint16_t dirty_cw = 0x0F7F;
    __asm__ volatile("fldcw %0" : : "m"(dirty_cw));

    __asm__ volatile("fld1");
    __asm__ volatile("fld1");
    __asm__ volatile("fld1");
    __asm__ volatile("fld1");
    __asm__ volatile("fld1");

    /* Sanity-check the pre-finit state. */
    uint16_t cw_before = fnstcw();
    uint16_t top_before = (fnstsw() >> 11) & 7;
    uint16_t tw_before = fnstenv_tw();
    if (cw_before != 0x0F7F) {
        printf("FAIL  test_finit_resets_all: setup CW — got=0x%04x expected=0x0F7F\n", cw_before);
        failures++;
    }
    if (top_before != 3) {
        printf("FAIL  test_finit_resets_all: setup TOP — got=%u expected=3\n", top_before);
        failures++;
    }
    /* TW after 5 pushes from TOP=0: physical slots 7,6,5,4,3 are valid (00),
     * slots 0,1,2 are empty (11).  TW = 0000_00_11_11_11 in physical-low-bit
     * order = 0x003F.  After pushes, FNSTENV may flush the deferred tag
     * state; either 0x003F (flushed) or different cache representation —
     * we just need it != 0xFFFF (empty). */
    if (tw_before == 0xFFFF) {
        printf("FAIL  test_finit_resets_all: setup TW — got=0xFFFF expected != all-empty\n");
        failures++;
    }

    /* Now the adversarial cross-path: FNINIT may fall through to stock. */
    __asm__ volatile("fninit");

    /* Verify reset.  If stock wrote CW/SW/TW at wrong offsets, our
     * FNSTCW/FNSTSW/FNSTENV would read stale dirty values. */
    check_eq_u16("test_finit_resets_all: CW=0x037F", fnstcw(), 0x037F);
    check_eq_u16("test_finit_resets_all: SW=0 (TOP=0)", fnstsw(), 0x0000);
    check_eq_u16("test_finit_resets_all: TW=0xFFFF (all empty)", fnstenv_tw(), 0xFFFF);
}

/* ── Verify post-finit FLD lands at physical slot 7 (TOP wraps from 0). */
static void test_finit_fld_after(void) {
    __asm__ volatile("fninit");
    __asm__ volatile("fld1");

    /* FLD pre-decrements TOP so TOP goes from 0 to 7. */
    uint16_t top = (fnstsw() >> 11) & 7;
    check_eq_u16("test_finit_fld_after: TOP=7 after FLD", top, 7);

    /* Verify the value — read back ST(0) via fst. */
    double x = 0.0;
    __asm__ volatile("fstpl %0" : "=m"(x));
    if (x != 1.0) {
        printf("FAIL  test_finit_fld_after: ST(0) value — got=%g expected=1.0\n", x);
        failures++;
    } else {
        printf("PASS  test_finit_fld_after: ST(0)=1.0\n");
    }
}

int main(void) {
    test_finit_resets_all();
    test_finit_fld_after();

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
