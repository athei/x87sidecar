/*
 * test_fbld.c — Tests for FBLD m80bcd (DF /4).
 *
 * BCD layout (10 bytes, little-endian):
 *   bytes[0..8] : 18 packed BCD digits (2 per byte, low byte = lowest digits)
 *                 each byte: bits[7:4] = high digit, bits[3:0] = low digit
 *   byte[9]     : bit 7 = sign (1 = negative), bits[6:0] reserved (zero)
 *
 * x87 spec: load 18-digit BCD as f80, push onto stack. Range up to ±10^18-1.
 *
 * Build: clang -arch x86_64 -O0 -o test_fbld test_fbld.c
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static double do_fbld(const uint8_t bcd[10]) {
    double r;
    __asm__ volatile(
        "fbld  %1\n\t"
        "fstpl %0\n"
        : "=m"(r)
        : "m"(*(const uint8_t (*)[10])bcd)
        : "st");
    return r;
}

static void check(const char* name, const uint8_t bcd[10], double expected) {
    double got = do_fbld(bcd);
    /* Bit-exact compare so -0 vs +0 (and any NaN payload) is caught. */
    uint64_t got_bits, exp_bits;
    memcpy(&got_bits, &got, 8);
    memcpy(&exp_bits, &expected, 8);
    int ok = (got_bits == exp_bits);
    printf("%s  %-40s  got=%.17g (bits=%016llx)  expected=%.17g (bits=%016llx)\n",
           ok ? "PASS" : "FAIL", name, got, (unsigned long long)got_bits, expected,
           (unsigned long long)exp_bits);
    if (!ok)
        failures++;
}

int main(void) {
    /* +0  → all zero, no sign bit */
    {
        uint8_t b[10] = {0};
        check("fbld(+0)", b, 0.0);
    }

    /* -0  → sign bit set, all-zero digits */
    {
        uint8_t b[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0x80};
        check("fbld(-0)", b, -0.0);
    }

    /* +42  (0x42 = digits {4,2}) — same encoding mmo uses */
    {
        uint8_t b[10] = {0x42};
        check("fbld(+42)", b, 42.0);
    }

    /* +1 */
    {
        uint8_t b[10] = {0x01};
        check("fbld(+1)", b, 1.0);
    }

    /* -1 */
    {
        uint8_t b[10] = {0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0x80};
        check("fbld(-1)", b, -1.0);
    }

    /* +9999 (digits 9,9,9,9 in bytes 0-1: byte0=0x99, byte1=0x99) */
    {
        uint8_t b[10] = {0x99, 0x99};
        check("fbld(+9999)", b, 9999.0);
    }

    /* +123456 */
    {
        uint8_t b[10] = {0x56, 0x34, 0x12};
        check("fbld(+123456)", b, 123456.0);
    }

    /* -987654321 */
    {
        uint8_t b[10] = {0x21, 0x43, 0x65, 0x87, 0x09, 0, 0, 0, 0, 0x80};
        check("fbld(-987654321)", b, -987654321.0);
    }

    /* +10^17 = 100000000000000000  (digit 17 = 1) */
    {
        uint8_t b[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0x10};
        check("fbld(+10^17)", b, 1e17);
    }

    /* +999999999999999999 = 10^18 - 1 (all 18 digits = 9) — fits in i64 */
    {
        uint8_t b[10] = {0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99};
        check("fbld(+999999999999999999)", b, 999999999999999999.0);
    }

    /* -10^18+1 = same all-nines, sign set */
    {
        uint8_t b[10] = {0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x80};
        check("fbld(-999999999999999999)", b, -999999999999999999.0);
    }

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
