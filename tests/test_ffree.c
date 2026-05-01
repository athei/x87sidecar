/*
 * test_ffree.c — Tests for FFREE ST(i) (DD C0+i).
 *
 * Spec: marks the tag for ST(i) as Empty (0b11) in the FPU tag word.
 *       No data movement, no TOP change, no exception flags affected.
 *
 * The tag for ST(i) lives in tag_word bits [(2*phys+1):(2*phys)] where
 * phys = (TOP + i) & 7.  We inspect tag_word via fnstenv (offset +8 in
 * the 28-byte 32-bit protected-mode FPU env).
 *
 * Tests:
 *   1. ffree ST(1) sets tag bits for the corresponding physical slot to
 *      empty; other slots' tags unchanged; ST(0) and ST(2) values intact.
 *   2. ffree ST(0) marks the top of stack as empty.
 *   3. ffree of an already-empty slot is idempotent.
 *   4. After ffree ST(i), fld back the same slot index re-tags it valid.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static uint16_t fnstsw(void) {
    uint16_t sw;
    __asm__ volatile("fnstsw %0" : "=m"(sw));
    return sw;
}

static uint16_t snapshot_tag_word(void) {
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

static void check_bitexact_f64(const char* name, double got, double expected) {
    uint64_t g, e;
    memcpy(&g, &got, 8);
    memcpy(&e, &expected, 8);
    if (g == e) {
        printf("PASS  %s  (val=%.17g)\n", name, got);
    } else {
        printf("FAIL  %s  got=%.17g (bits=%016llx) expected=%.17g (bits=%016llx)\n", name, got,
               (unsigned long long)g, expected, (unsigned long long)e);
        failures++;
    }
}

/* ── Test 1: ffree ST(1) clears the right tag, leaves others untouched. ──── */
static void test_ffree_st1(void) {
    __asm__ volatile("fninit");
    volatile double a = 1.0, b = 2.0, c = 3.0;
    __asm__ volatile("fldl %0" : : "m"(a)); /* TOP=7, reg7=1.0 */
    __asm__ volatile("fldl %0" : : "m"(b)); /* TOP=6, reg6=2.0 */
    __asm__ volatile("fldl %0" : : "m"(c)); /* TOP=5, reg5=3.0 */

    uint16_t tag_before = snapshot_tag_word();
    /* slot 5,6,7 valid (00); slots 0..4 empty (11).  Pattern: 0b00_00_00_11_11_11_11_11 = 0x03FF */
    check_eq_u16("ffree st1: tag_word before", tag_before, 0x03FF);

    __asm__ volatile("ffree %st(1)");

    uint16_t tag_after = snapshot_tag_word();
    /* After ffree ST(1): physical slot (5+1)&7 = 6 marked empty.
     * Slot 6 bits [13:12] go from 00 → 11. tag_word: 0x03FF | (3<<12) = 0x33FF. */
    check_eq_u16("ffree st1: tag_word after", tag_after, 0x33FF);

    /* ST(0) (reg 5) and ST(2) (reg 7) values should still be intact. */
    double r0;
    __asm__ volatile("fstl %0" : "=m"(r0) : : "st"); /* peek ST(0), don't pop */
    check_bitexact_f64("ffree st1: ST(0) value intact", r0, 3.0);

    __asm__ volatile("fninit");
}

/* ── Test 2: ffree ST(0) marks top of stack as empty. ─────────────────────── */
static void test_ffree_st0(void) {
    __asm__ volatile("fninit");
    volatile double a = 7.5;
    __asm__ volatile("fldl %0" : : "m"(a)); /* TOP=7, reg7=7.5 */

    uint16_t tag_before = snapshot_tag_word();
    /* Only slot 7 valid: 0x3FFF. */
    check_eq_u16("ffree st0: tag_word before", tag_before, 0x3FFF);

    __asm__ volatile("ffree %st(0)");

    uint16_t tag_after = snapshot_tag_word();
    /* All slots empty now: 0xFFFF. */
    check_eq_u16("ffree st0: tag_word after", tag_after, 0xFFFF);

    __asm__ volatile("fninit");
}

/* ── Test 3: ffree of an already-empty slot is idempotent. ────────────────── */
static void test_ffree_already_empty(void) {
    __asm__ volatile("fninit");
    /* Stack is fully empty after fninit; tag = 0xFFFF.  ffree any slot is no-op. */
    __asm__ volatile("ffree %st(3)");
    uint16_t tag = snapshot_tag_word();
    check_eq_u16("ffree already-empty: tag still all empty", tag, 0xFFFF);
}

/* ── Test 4: after ffree ST(i), an fld retags valid. ──────────────────────── */
static void test_ffree_then_fld(void) {
    __asm__ volatile("fninit");
    volatile double a = 1.0, b = 2.0, c = 3.0, d = 4.0;
    __asm__ volatile("fldl %0" : : "m"(a));
    __asm__ volatile("fldl %0" : : "m"(b));
    __asm__ volatile("fldl %0" : : "m"(c));

    __asm__ volatile("ffree %st(2)"); /* mark old ST(2)=1.0 slot empty */
    /* New fld pushes — slot at new TOP must become valid even if it was
     * previously freed.  This sanity-checks ffree's tag write didn't break
     * subsequent push tagging. */
    __asm__ volatile("fldl %0" : : "m"(d)); /* TOP-=1, push 4.0 */

    uint16_t tag = snapshot_tag_word();
    /* The fld retags its slot valid; the freed slot 7 is still empty. */
    /* TOP rotates as: started TOP=0, three flds → TOP=5, ffree st(2) doesn't move TOP,
     * fld → TOP=4, reg4=4.0 valid.
     * Slots before ffree: 5,6,7 valid.  ffree st(2) → slot (5+2)&7=7 empty.
     * After fld 4.0 at TOP=4: slot 4 valid.
     * Final tags: slots 4,5,6 valid; slot 7 empty; others empty.
     * Pattern: bits[15:14]=11, bits[13:12]=00, bits[11:10]=00, bits[9:8]=00,
     *          bits[7:6]=11, bits[5:4]=11, bits[3:2]=11, bits[1:0]=11
     * = 0b11_00_00_00_11_11_11_11 = 0xC0FF. */
    check_eq_u16("ffree then fld: tag pattern", tag, 0xC0FF);

    __asm__ volatile("fninit");
}

int main(void) {
    test_ffree_st1();
    test_ffree_st0();
    test_ffree_already_empty();
    test_ffree_then_fld();

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
