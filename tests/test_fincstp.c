/*
 * test_fincstp.c — Tests for FINCSTP (D9 F7).
 *
 * Spec: TOP = (TOP + 1) & 7.  No tag change, no data movement.
 *       C1 cleared; C0/C2/C3 undefined.
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

/* ── Test 1: TOP increments by 1 (mod 8). ─────────────────────────────────── */
static void test_top_rotates(void) {
    __asm__ volatile("fninit"); /* TOP=0 */
    __asm__ volatile("fld1");   /* TOP=7 */
    uint16_t before = top_of(fnstsw());
    __asm__ volatile("fincstp"); /* TOP=0 (wraps) */
    uint16_t after = top_of(fnstsw());

    check_eq_u16("test_top_rotates: TOP before fincstp", before, 7);
    check_eq_u16("test_top_rotates: TOP after fincstp", after, 0);

    /* Cleanup: TOP=0 means ST(0) is whatever was at register 0 (empty).
     * fninit drops everything. */
    __asm__ volatile("fninit");
}

/* ── Test 2: data registers unchanged; ST(0) view shifts the other way. ──── */
static void test_stack_view_shifts(void) {
    __asm__ volatile("fninit");
    /* Push 1, 2, 3 → ST(0)=3, ST(1)=2, ST(2)=1, TOP=5. */
    volatile double a = 1.0, b = 2.0, c = 3.0;
    __asm__ volatile("fldl %0" : : "m"(a));
    __asm__ volatile("fldl %0" : : "m"(b));
    __asm__ volatile("fldl %0" : : "m"(c));

    /* fincstp: TOP+=1 (now 6).  ST(0) was 3.0 (reg 5) — now ST(7).
     * Old ST(1)=2.0 (reg 6) is now ST(0).
     * Old ST(2)=1.0 (reg 7) is now ST(1). */
    __asm__ volatile("fincstp");

    double r0, r1;
    __asm__ volatile("fstpl %0" : "=m"(r0) : : "st"); /* pops new ST(0) */
    __asm__ volatile("fstpl %0" : "=m"(r1) : : "st"); /* pops next */

    check_bitexact_f64("test_stack_view_shifts: old ST(1) is now ST(0)", r0, 2.0);
    check_bitexact_f64("test_stack_view_shifts: old ST(2) is now ST(1)", r1, 1.0);

    __asm__ volatile("fninit");
}

/* ── Test 3: fincstp + fdecstp round-trips a value. ───────────────────────── */
static void test_round_trip(void) {
    __asm__ volatile("fninit");
    volatile double x = 42.42;
    __asm__ volatile("fldl %0" : : "m"(x));

    __asm__ volatile("fincstp");
    __asm__ volatile("fdecstp");

    double r;
    __asm__ volatile("fstpl %0" : "=m"(r) : : "st");

    check_bitexact_f64("test_round_trip: fincstp+fdecstp identity", r, 42.42);
}

/* ── Test 4: 8x fincstp wraps back to start TOP. ──────────────────────────── */
static void test_wraparound(void) {
    __asm__ volatile("fninit");
    uint16_t start = top_of(fnstsw());
    for (int i = 0; i < 8; i++) {
        __asm__ volatile("fincstp");
    }
    uint16_t end = top_of(fnstsw());
    check_eq_u16("test_wraparound: 8x fincstp returns to start TOP", end, start);
}

int main(void) {
    test_top_rotates();
    test_stack_view_shifts();
    test_round_trip();
    test_wraparound();

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
