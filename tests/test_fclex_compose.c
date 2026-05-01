/*
 * test_fclex_compose.c — adversarial composition test for FCLEX/FNCLEX.
 *
 * Guards against an m108-style internal-offset bug (per
 * project_native_rosetta_lazy_f80.md) where stock might write SW at a
 * different byte offset than our X87State expects.  Pattern:
 *   1. Set SW.exception bits via inline ops (writes our internal SW @ 0x02).
 *   2. Run FNCLEX (fall-through to stock if not in is_handled_x87).
 *   3. Read SW back via inline FNSTSW (reads our internal SW @ 0x02).
 *   4. Assert IE bit cleared and TOP unchanged.
 *
 * If stock RMW'd SW at a different internal offset (e.g., 4 instead of 2,
 * mistakenly using the m28byte-padded layout), inline FNSTSW would still
 * read the dirty bits at offset 2 → IE stays set → test FAILS.
 *
 * Bonus: the test is also valid when fclex stays inline (existing path);
 * passes equally in both cases because the layout assumption is correct.
 */
#include <stdint.h>
#include <stdio.h>

static int failures = 0;

static uint16_t fnstsw(void) {
    uint16_t sw;
    __asm__ volatile("fnstsw %0" : "=m"(sw));
    return sw;
}

/* Inject exception bits into SW via fldenv (the only way to set
 * them from user mode without depending on fdiv-by-zero etc.). */
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
        printf("PASS  %s  (got=0x%04x)\n", name, got);
    } else {
        printf("FAIL  %s  got=0x%04x expected=0x%04x\n", name, got, expected);
        failures++;
    }
}

/* ── Adversarial test 1: distinctive bit pattern in low byte (0x55).
 * Bits 0,2,4,6 set so a 1-byte-offset misread would give a clearly
 * different value (0x55 → reading from offset 4 of our state would
 * land in TW low byte = 0xFF after fninit). */
static void test_fclex_low_byte_pattern(void) {
    __asm__ volatile("fninit");
    inject_status_word(0x0055); /* SW low byte = 0x55: bits 0,2,4,6 */

    uint16_t sw_before = fnstsw();
    if ((sw_before & 0x007F) != 0x0055) {
        printf("FAIL  test_fclex_low_byte_pattern: setup — sw_before=0x%04x expected low=0x55\n",
               sw_before);
        failures++;
        return;
    }

    __asm__ volatile("fnclex");

    uint16_t sw_after = fnstsw();
    /* All clearable bits cleared (0x7F mask), TOP preserved (was 0). */
    check_eq_u16("test_fclex_low_byte_pattern: clearable bits cleared", sw_after & 0x7F, 0x0000);
    check_eq_u16("test_fclex_low_byte_pattern: TOP unchanged", (sw_after >> 11) & 7, 0);
}

/* ── Adversarial test 2: TOP=5 distinctive in SW[13:11], plus bits 0..6.
 * If stock wrote SW at the wrong offset, TOP would not survive the
 * round-trip. */
static void test_fclex_with_top(void) {
    __asm__ volatile("fninit");
    /* Push 3 values to set TOP=5. */
    __asm__ volatile("fld1");
    __asm__ volatile("fld1");
    __asm__ volatile("fld1");

    uint16_t top_before = (fnstsw() >> 11) & 7;
    if (top_before != 5) {
        printf("FAIL  test_fclex_with_top: setup — TOP=%u expected 5\n", top_before);
        failures++;
        return;
    }

    /* Now inject exception bits while keeping TOP=5. */
    inject_status_word(0x002A); /* bits 1,3,5: distinctive non-trivial pattern */

    uint16_t sw_before = fnstsw();
    if ((sw_before & 0x007F) != 0x002A) {
        printf("FAIL  test_fclex_with_top: setup — sw_before=0x%04x low=0x%02x\n", sw_before,
               sw_before & 0x7F);
        failures++;
    }

    __asm__ volatile("fnclex");

    uint16_t sw_after = fnstsw();
    check_eq_u16("test_fclex_with_top: clearable bits cleared", sw_after & 0x7F, 0x0000);
    check_eq_u16("test_fclex_with_top: TOP=5 preserved", (sw_after >> 11) & 7, 5);

    __asm__ volatile("fstp %%st(0)" : : : "st");
    __asm__ volatile("fstp %%st(0)" : : : "st");
    __asm__ volatile("fstp %%st(0)" : : : "st");
}

int main(void) {
    test_fclex_low_byte_pattern();
    test_fclex_with_top();

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
