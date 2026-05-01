/*
 * test_fdecstp.c — Tests for FDECSTP (D9 F6).
 *
 * Spec: TOP = (TOP - 1) & 7.  No tag change, no data movement.
 *       C1 is cleared; C0/C2/C3 undefined (per Intel SDM).
 *
 * Tests:
 *   1. TOP rotates correctly (visible via fnstsw bits 13:11).
 *   2. ST values shift in the architectural view (ST(1) before becomes
 *      ST(2) after).
 *   3. fdecstp + fincstp is identity (round-trips ST(0) bit-exact).
 *   4. Wraparound: fdecstp from TOP=0 gives TOP=7.
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

static uint16_t top_of(uint16_t sw) {
    return (sw >> 11) & 7;
}

static void check_eq_u16(const char* name, uint16_t got, uint16_t expected) {
    if (got == expected) {
        printf("PASS  %s  (got=%u)\n", name, got);
    } else {
        printf("FAIL  %s  got=%u expected=%u\n", name, got, expected);
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

/* ── Test 1: TOP decrements by 1. ─────────────────────────────────────────── */
static void test_top_rotates(void) {
    __asm__ volatile("fninit");
    /* TOP=0 after fninit. */
    uint16_t before = top_of(fnstsw());
    __asm__ volatile("fdecstp");
    uint16_t after = top_of(fnstsw());

    check_eq_u16("test_top_rotates: TOP before fdecstp", before, 0);
    check_eq_u16("test_top_rotates: TOP after fdecstp", after, 7);
}

/* ── Test 2: data registers unchanged; ST(0) view shifts. ─────────────────── */
static void test_stack_view_shifts(void) {
    __asm__ volatile("fninit");
    /* Push 1, 2, 3 so ST(0)=3.0, ST(1)=2.0, ST(2)=1.0. */
    volatile double a = 1.0, b = 2.0, c = 3.0;
    __asm__ volatile("fldl %0" : : "m"(a));
    __asm__ volatile("fldl %0" : : "m"(b));
    __asm__ volatile("fldl %0" : : "m"(c));

    /* fdecstp: TOP-=1, so the architectural view shifts.  A previously-empty
     * slot appears at ST(0); old ST(0)=3.0 is now ST(1); old ST(1)=2.0 is
     * now ST(2); old ST(2)=1.0 is now ST(3). */
    __asm__ volatile("fdecstp");

    /* Pull ST(1), ST(2), ST(3) and verify the shift. */
    double r1, r2, r3;
    __asm__ volatile(
        "fxch %%st(1)\n\t"
        "fstpl %0\n\t"
        : "=m"(r1)
        :
        : "st");
    __asm__ volatile(
        "fxch %%st(1)\n\t" /* st(2) was st(2) before ↑ pop, now st(1) */
        "fstpl %0\n\t"
        : "=m"(r2)
        :
        : "st");
    __asm__ volatile(
        "fxch %%st(1)\n\t"
        "fstpl %0\n\t"
        : "=m"(r3)
        :
        : "st");

    check_bitexact_f64("test_stack_view_shifts: old ST(0) is now ST(1)", r1, 3.0);
    check_bitexact_f64("test_stack_view_shifts: old ST(1) is now ST(2)", r2, 2.0);
    check_bitexact_f64("test_stack_view_shifts: old ST(2) is now ST(3)", r3, 1.0);

    /* Drain the rest. */
    __asm__ volatile("fninit");
}

/* ── Test 3: fdecstp + fincstp round-trips a value. ───────────────────────── */
static void test_round_trip(void) {
    __asm__ volatile("fninit");
    volatile double x = 12345.6789;
    __asm__ volatile("fldl %0" : : "m"(x));

    __asm__ volatile("fdecstp");
    __asm__ volatile("fincstp");

    double r;
    __asm__ volatile("fstpl %0" : "=m"(r) : : "st");

    check_bitexact_f64("test_round_trip: fdecstp+fincstp identity", r, 12345.6789);
}

/* ── Test 4: many fdecstp wraps cleanly back to original TOP after 8. ────── */
static void test_wraparound(void) {
    __asm__ volatile("fninit");
    uint16_t start = top_of(fnstsw());
    for (int i = 0; i < 8; i++) {
        __asm__ volatile("fdecstp");
    }
    uint16_t end = top_of(fnstsw());
    check_eq_u16("test_wraparound: 8x fdecstp returns to start TOP", end, start);
}

int main(void) {
    test_top_rotates();
    test_stack_view_shifts();
    test_round_trip();
    test_wraparound();

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
