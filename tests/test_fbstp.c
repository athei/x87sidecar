/*
 * test_fbstp.c — Tests for FBSTP m80bcd (DF /6).
 *
 * BCD layout (10 bytes, little-endian by digit-pair):
 *   bytes[0..8] : 18 packed BCD digits (low-to-high), 2 per byte
 *                  byte[i] = (digit[2i+1] << 4) | digit[2i]
 *   byte[9]     : bit 7 = sign (1 = negative), bits[6:0] reserved (zero)
 *
 * x87 spec (Intel SDM Vol. 1 §8.5):
 *   - Round ST(0) to integer per CW.RC.
 *   - If |rounded| >= 10^18 OR ST(0) is NaN/±inf → write the BCD-indefinite
 *     pattern (memory bytes 0..6 = 0x00, byte 7 = 0xC0, bytes 8-9 = 0xFF).
 *   - Else split |rounded| into 18 BCD digits and write the sign byte.
 *   - Pop ST(0).
 *
 * We test all 4 rounding modes by setting CW.RC via fldcw before each fbstp.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

/* x86 CW.RC values (bits 11:10): */
#define RC_NEAREST 0x0000
#define RC_DOWN    0x0400
#define RC_UP      0x0800
#define RC_ZERO    0x0C00

static const uint8_t INDEF_BCD[10] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0xFF
};

static void do_fbstp(double in, uint8_t out[10]) {
    /* Wrapper: FLDL  in;  FBSTP  out  (also pops ST(0)). */
    __asm__ volatile (
        "fldl  %1\n\t"
        "fbstp %0\n"
        : "=m"(*(uint8_t (*)[10])out)
        : "m"(in)
        : "st"
    );
}

static void set_rc(uint16_t rc_bits) {
    /* CW = 0x037F | rc_bits  (mask all exceptions, default precision 64-bit) */
    uint16_t cw = 0x037F | rc_bits;
    __asm__ volatile ("fldcw %0" : : "m"(cw));
}

static void reset_cw(void) {
    uint16_t cw = 0x037F;
    __asm__ volatile ("fldcw %0" : : "m"(cw));
}

static void check_bcd(const char *name, const uint8_t got[10], const uint8_t expected[10]) {
    if (memcmp(got, expected, 10) == 0) {
        printf("PASS  %-50s\n", name);
        return;
    }
    printf("FAIL  %-50s\n", name);
    printf("        got =");
    for (int i = 9; i >= 0; i--) printf(" %02x", got[i]);
    printf("\n        exp =");
    for (int i = 9; i >= 0; i--) printf(" %02x", expected[i]);
    printf("\n");
    failures++;
}

/* ── Build a 10-byte BCD buffer from a magnitude (uint64) and sign. ────── */
static void encode_bcd(uint64_t mag, int negative, uint8_t out[10]) {
    memset(out, 0, 10);
    for (int i = 0; i < 9; i++) {
        unsigned d0 = mag % 10; mag /= 10;
        unsigned d1 = mag % 10; mag /= 10;
        out[i] = (uint8_t)((d1 << 4) | d0);
    }
    if (negative) out[9] = 0x80;
}

static void check_signed(const char *name, double in, int64_t expected_int) {
    uint8_t got[10];
    do_fbstp(in, got);
    uint8_t expected[10];
    int neg = (expected_int < 0);
    uint64_t mag = neg ? (uint64_t)(-expected_int) : (uint64_t)expected_int;
    encode_bcd(mag, neg, expected);
    check_bcd(name, got, expected);
}

static void check_indef(const char *name, double in) {
    uint8_t got[10];
    do_fbstp(in, got);
    check_bcd(name, got, INDEF_BCD);
}

/* ── Round-nearest-even (default) baseline ───────────────────────────────── */
static void test_rc_nearest_baseline(void) {
    set_rc(RC_NEAREST);

    /* Trivial integers. */
    check_signed("nearest +1",       1.0,  1);
    check_signed("nearest -1",      -1.0, -1);
    check_signed("nearest +42",     42.0, 42);
    check_signed("nearest +123456", 123456.0, 123456);
    check_signed("nearest -987654321", -987654321.0, -987654321);

    /* Half-way: round to even. */
    check_signed("nearest +1.5 → 2",  1.5,  2);
    check_signed("nearest +2.5 → 2",  2.5,  2);
    check_signed("nearest -1.5 → -2", -1.5, -2);
    check_signed("nearest -2.5 → -2", -2.5, -2);

    /* Quarter values. */
    check_signed("nearest +0.4 → 0",  0.4,  0);
    check_signed("nearest +0.6 → 1",  0.6,  1);
    check_signed("nearest -0.6 → -1", -0.6, -1);

    reset_cw();
}

/* ── RC=1 (round toward -inf, "down/floor") ──────────────────────────────── */
static void test_rc_down(void) {
    set_rc(RC_DOWN);

    check_signed("down +1.5 → 1",  1.5,  1);
    check_signed("down +2.5 → 2",  2.5,  2);
    check_signed("down -1.5 → -2", -1.5, -2);
    check_signed("down -2.5 → -3", -2.5, -3);
    check_signed("down +0.9 → 0",  0.9,  0);
    check_signed("down -0.1 → -1", -0.1, -1);

    reset_cw();
}

/* ── RC=2 (round toward +inf, "up/ceil") ─────────────────────────────────── */
static void test_rc_up(void) {
    set_rc(RC_UP);

    check_signed("up +1.5 → 2",  1.5,  2);
    check_signed("up +2.5 → 3",  2.5,  3);
    check_signed("up -1.5 → -1", -1.5, -1);
    check_signed("up -2.5 → -2", -2.5, -2);
    check_signed("up +0.1 → 1",  0.1,  1);
    /* up -0.9: rounds toward +inf → -0.0.  Hardware preserves the sign,
     * so the BCD result has sign byte 0x80 (negative zero). */
    {
        uint8_t got[10];
        do_fbstp(-0.9, got);
        uint8_t exp[10] = {0,0,0,0,0,0,0,0,0,0x80};
        check_bcd("up -0.9 → -0 (sign byte 0x80)", got, exp);
    }

    reset_cw();
}

/* ── RC=3 (round toward zero, "truncate") ────────────────────────────────── */
static void test_rc_zero(void) {
    set_rc(RC_ZERO);

    check_signed("trunc +1.5 → 1",  1.5,  1);
    check_signed("trunc +2.5 → 2",  2.5,  2);
    check_signed("trunc -1.5 → -1", -1.5, -1);
    check_signed("trunc -2.5 → -2", -2.5, -2);
    check_signed("trunc +0.9 → 0",  0.9,  0);
    /* trunc -0.9: truncate toward zero → -0.0.  Sign byte 0x80. */
    {
        uint8_t got[10];
        do_fbstp(-0.9, got);
        uint8_t exp[10] = {0,0,0,0,0,0,0,0,0,0x80};
        check_bcd("trunc -0.9 → -0 (sign byte 0x80)", got, exp);
    }

    reset_cw();
}

/* ── Zero handling — including -0.0 sign byte ────────────────────────────── */
static void test_zeros(void) {
    set_rc(RC_NEAREST);

    /* +0.0 → all zero */
    {
        uint8_t got[10];
        do_fbstp(0.0, got);
        uint8_t exp[10] = {0};
        check_bcd("+0.0 → all zero", got, exp);
    }
    /* -0.0 → byte 9 = 0x80 (sign), digits all zero */
    {
        uint8_t got[10];
        do_fbstp(-0.0, got);
        uint8_t exp[10] = {0,0,0,0,0,0,0,0,0,0x80};
        check_bcd("-0.0 → sign byte 0x80", got, exp);
    }

    reset_cw();
}

/* ── Full range — exactly representable in f64 (mantissa < 2^53) ─────────── */
static void test_full_range(void) {
    set_rc(RC_NEAREST);

    /* 2^53 - 1 = 9007199254740991 — largest int exactly representable as f64.
     * (10^18 - 1 isn't representable; it rounds up to 10^18 = indefinite.) */
    check_signed("+(2^53-1)", 9007199254740991.0,  9007199254740991LL);
    check_signed("-(2^53-1)", -9007199254740991.0, -9007199254740991LL);

    /* 16-digit value mid-range. */
    check_signed("+1234567890123456",   1234567890123456.0,   1234567890123456LL);
    check_signed("-1234567890123456",  -1234567890123456.0,  -1234567890123456LL);

    reset_cw();
}

/* ── Indefinite cases: NaN, ±inf, |val| >= 10^18 ─────────────────────────── */
static void test_indefinite(void) {
    set_rc(RC_NEAREST);

    /* NaN */
    {
        union { uint64_t u; double d; } nan = { .u = 0x7FF8000000000001ULL };
        check_indef("NaN → indefinite", nan.d);
    }
    /* +inf */
    {
        union { uint64_t u; double d; } pinf = { .u = 0x7FF0000000000000ULL };
        check_indef("+inf → indefinite", pinf.d);
    }
    /* -inf */
    {
        union { uint64_t u; double d; } ninf = { .u = 0xFFF0000000000000ULL };
        check_indef("-inf → indefinite", ninf.d);
    }
    /* |val| == 1e18 (exactly representable as a double) */
    check_indef("+1e18 → indefinite", 1e18);
    check_indef("-1e18 → indefinite", -1e18);
    check_indef("+1e19 → indefinite", 1e19);

    reset_cw();
}

int main(void) {
    test_rc_nearest_baseline();
    test_rc_down();
    test_rc_up();
    test_rc_zero();
    test_zeros();
    test_full_range();
    test_indefinite();

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
