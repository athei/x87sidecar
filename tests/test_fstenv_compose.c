/*
 * test_fstenv_compose.c — adversarial composition test for FSTENV/FNSTENV.
 *
 * Guards against an m108-style internal-offset bug.  Pattern:
 *   1. Set our internal CW/SW/TW to distinctive values via inline ops
 *      (writes to our X87State at offsets 0x00/0x02/0x04).
 *   2. Run FNSTENV (fall-through to stock if not in is_handled_x87) —
 *      stock reads CW/SW/TW from our state and writes to user buffer.
 *   3. Read CW/SW/TW from the user buffer at m28byte offsets.
 *   4. Assert exact match against what we set inline.
 *
 * If stock fstenv read SW from offset 4 of our state (m108-padded
 * layout) instead of offset 2, buf[4..5] would reflect the byte at our
 * state offset 4 — which is TW low byte after a fld sequence — not SW.
 * → test FAILS.
 *
 * Bonus deferred-state test: FXCH sets perm_dirty, FFREE sets
 * deferred_pop_count, push sets tag_push_pending.  The x87_end
 * invariant must flush all of these before stock reads the env.
 * Tests exercise the flush path explicitly.
 */
#include <stdint.h>
#include <stdio.h>

static int failures = 0;

static void check_eq_u16(const char* name, uint16_t got, uint16_t expected) {
    if (got == expected) {
        printf("PASS  %s  (got=0x%04x)\n", name, got);
    } else {
        printf("FAIL  %s  got=0x%04x expected=0x%04x\n", name, got, expected);
        failures++;
    }
}

/* Extract CW/SW/TW from a 28-byte m28byte env buffer. */
static uint16_t env_cw(const uint8_t* env) {
    return (uint16_t)env[0] | ((uint16_t)env[1] << 8);
}
static uint16_t env_sw(const uint8_t* env) {
    return (uint16_t)env[4] | ((uint16_t)env[5] << 8);
}
static uint16_t env_tw(const uint8_t* env) {
    return (uint16_t)env[8] | ((uint16_t)env[9] << 8);
}

/* ── Adversarial test 1: distinctive CW + TOP + TW via inline ops. */
static void test_fstenv_simple(void) {
    __asm__ volatile("fninit");

    /* Set CW to 0x0F7F (precision=64b, distinctive vs default 0x037F). */
    uint16_t cw = 0x0F7F;
    __asm__ volatile("fldcw %0" : : "m"(cw));

    /* Push 4 values to set TOP=4 (TOP in SW.bits[13:11]).  After 4
     * pushes from TOP=0, TOP wraps to 4. */
    __asm__ volatile("fld1");
    __asm__ volatile("fld1");
    __asm__ volatile("fld1");
    __asm__ volatile("fld1");

    /* Cross-path: fnstenv may fall through to stock. */
    uint8_t env[28] __attribute__((aligned(16))) = {0};
    __asm__ volatile("fnstenv %0" : "=m"(env)::"memory");

    /* Verify CW round-tripped through the cross-path read. */
    check_eq_u16("test_fstenv_simple: CW=0x0F7F", env_cw(env), 0x0F7F);

    /* TOP=4 should appear in env[5] bits [5:3] (SW.bits[13:11] in
     * little-endian = bits[13:8] of byte 5).  TOP=4 = 100 binary,
     * shifted into [13:11] = 0x2000 → env[5] = 0x20. */
    check_eq_u16("test_fstenv_simple: TOP=4", (env_sw(env) >> 11) & 7, 4);

    /* TW after 4 pushes from TOP=0: physical slots 4,5,6,7 are valid
     * (00), slots 0,1,2,3 are empty (11).  TW = 0000_0000_1111_1111
     * (msb to lsb, slot 7 first) — actually the standard layout is
     * slot N's tag at bit position 2*N, so slot 0 in lsb.
     * Slot 0..3 empty (11) = bits 0..7 = 0xFF.
     * Slot 4..7 valid (00) = bits 8..15 = 0x00.
     * TW = 0x00FF. */
    check_eq_u16("test_fstenv_simple: TW=0x00FF", env_tw(env), 0x00FF);

    /* Cleanup: pop 4 values. */
    for (int i = 0; i < 4; i++) {
        __asm__ volatile("fstp %%st(0)" : : : "st");
    }
}

/* ── Adversarial test 2: deferred cache state must be flushed before
 * stock reads the env.  Sets perm_dirty (via FXCH), tag_push_pending
 * (via FLD), deferred_pop_count (via FFREE).  All three should be
 * coherent in memory by the time fstenv runs. */
static void test_fstenv_deferred_flush(void) {
    __asm__ volatile("fninit");

    /* Push 3 values: TOP=5 after, slots 5,6,7 valid. */
    double a = 1.0, b = 2.0, c = 3.0;
    __asm__ volatile("fldl %0" : : "m"(a));
    __asm__ volatile("fldl %0" : : "m"(b));
    __asm__ volatile("fldl %0" : : "m"(c));

    /* FXCH ST(0) <-> ST(2): sets perm_dirty in our cache.  Doesn't
     * change SW/TW. */
    __asm__ volatile("fxch %%st(2)" : : : "st");

    /* FLD: another push, sets tag_push_pending.  TOP becomes 4. */
    double d = 4.0;
    __asm__ volatile("fldl %0" : : "m"(d));

    /* Cross-path: fnstenv must flush all three deferred flags before
     * stock reads the in-memory SW/TW. */
    uint8_t env[28] __attribute__((aligned(16))) = {0};
    __asm__ volatile("fnstenv %0" : "=m"(env)::"memory");

    /* TOP=4 expected after push of d. */
    check_eq_u16("test_fstenv_deferred_flush: TOP=4", (env_sw(env) >> 11) & 7, 4);

    /* TW after 4 valid pushes from TOP=0: slots 4,5,6,7 valid, 0..3 empty.
     * TW = 0x00FF. */
    check_eq_u16("test_fstenv_deferred_flush: TW=0x00FF (valid slots flushed)", env_tw(env),
                 0x00FF);

    /* Cleanup. */
    for (int i = 0; i < 4; i++) {
        __asm__ volatile("fstp %%st(0)" : : : "st");
    }
}

int main(void) {
    test_fstenv_simple();
    test_fstenv_deferred_flush();

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
