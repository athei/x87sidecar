/*
 * test_bcd_exact.c: FBLD and FBSTP with all 18 digits significant.
 *
 * An 18-digit packed BCD integer needs up to 60 bits, which the x87's
 * 64-bit significand holds exactly and a binary64 does not.  Two adjacent
 * pairs have to be exact:
 *
 *   fbld  m80bcd; fbstp m80bcd   a BCD copy
 *   fild  m64int; fbstp m80bcd   int64 to BCD, the conversion Delphi's
 *                                CurrToStr and FloatToDecimal use for
 *                                Currency values
 *
 * The integer source of the second pair is exact under every rounding
 * mode, and anything at or beyond 10^18 in magnitude stores the BCD
 * indefinite.  Each pair also runs inside a longer x87 run with a value
 * underneath it on the stack, which has to come out unchanged.
 *
 * Build: clang -arch x86_64 -O0 -o test_bcd_exact test_bcd_exact.c
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

#define CW_NEAREST 0x037F
#define CW_DOWN 0x077F
#define CW_UP 0x0B7F
#define CW_TRUNC 0x0F7F

static const uint8_t kIndef[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0xFF};

static void set_cw(uint16_t cw) {
    __asm__ volatile("fldcw %0" : : "m"(cw));
}

static void encode_bcd(uint64_t mag, int negative, uint8_t out[10]) {
    memset(out, 0, 10);
    for (int i = 0; i < 9; i++) {
        unsigned d0 = (unsigned)(mag % 10);
        mag /= 10;
        unsigned d1 = (unsigned)(mag % 10);
        mag /= 10;
        out[i] = (uint8_t)((d1 << 4) | d0);
    }
    if (negative) {
        out[9] = 0x80;
    }
}

static void bcd_copy(const uint8_t src[10], uint8_t dst[10]) {
    __asm__ volatile(
        "fbld  %1\n\t"
        "fbstp %0\n"
        : "=m"(*(uint8_t (*)[10])dst)
        : "m"(*(const uint8_t (*)[10])src)
        : "st");
}

static double bcd_copy_in_run(const uint8_t src[10], uint8_t dst[10]) {
    double below;
    __asm__ volatile(
        "fld1\n\t"
        "fbld  %2\n\t"
        "fbstp %0\n\t"
        "fstpl %1\n"
        : "=m"(*(uint8_t (*)[10])dst), "=m"(below)
        : "m"(*(const uint8_t (*)[10])src)
        : "st");
    return below;
}

static void int_to_bcd(const int64_t* src, uint8_t dst[10]) {
    __asm__ volatile(
        "fildll %1\n\t"
        "fbstp  %0\n"
        : "=m"(*(uint8_t (*)[10])dst)
        : "m"(*src)
        : "st");
}

static double int_to_bcd_in_run(const int64_t* src, uint8_t dst[10]) {
    double below;
    __asm__ volatile(
        "fld1\n\t"
        "fildll %2\n\t"
        "fbstp  %0\n\t"
        "fstpl  %1\n"
        : "=m"(*(uint8_t (*)[10])dst), "=m"(below)
        : "m"(*src)
        : "st");
    return below;
}

static void check_bcd(const char* shape, const char* name, const uint8_t got[10],
                      const uint8_t expected[10]) {
    if (memcmp(got, expected, 10) == 0) {
        printf("PASS  %-12s %s\n", shape, name);
        return;
    }
    printf("FAIL  %-12s %-34s got=", shape, name);
    for (int i = 9; i >= 0; i--) {
        printf("%02x", got[i]);
    }
    printf(" expected=");
    for (int i = 9; i >= 0; i--) {
        printf("%02x", expected[i]);
    }
    printf("\n");
    failures++;
}

static void check_below(const char* shape, const char* name, double below) {
    if (below != 1.0) {
        printf("FAIL  %-12s %-34s value underneath got=%g expected=1\n", shape, name, below);
        failures++;
    }
}

static void run_bcd_copy(const char* name, uint64_t mag, int negative) {
    uint8_t src[10], dst[10];
    encode_bcd(mag, negative, src);
    memset(dst, 0x5A, sizeof(dst));
    bcd_copy(src, dst);
    check_bcd("fbld pair", name, dst, src);
    memset(dst, 0x5A, sizeof(dst));
    check_below("fbld in run", name, bcd_copy_in_run(src, dst));
    check_bcd("fbld in run", name, dst, src);
}

static void run_int_to_bcd(const char* name, int64_t v) {
    uint8_t expected[10], dst[10];
    if (v <= -1000000000000000000LL || v >= 1000000000000000000LL) {
        memcpy(expected, kIndef, 10);
    } else {
        int neg = v < 0;
        encode_bcd(neg ? 0 - (uint64_t)v : (uint64_t)v, neg, expected);
    }
    memset(dst, 0x5A, sizeof(dst));
    int_to_bcd(&v, dst);
    check_bcd("fild pair", name, dst, expected);
    memset(dst, 0x5A, sizeof(dst));
    check_below("fild in run", name, int_to_bcd_in_run(&v, dst));
    check_bcd("fild in run", name, dst, expected);
}

static void run_all(void) {
    run_bcd_copy("123456789012345678", 123456789012345678ULL, 0);
    run_bcd_copy("-123456789012345678", 123456789012345678ULL, 1);
    run_bcd_copy("999999999999999999", 999999999999999999ULL, 0);
    run_bcd_copy("-999999999999999999", 999999999999999999ULL, 1);
    run_bcd_copy("2^53+1", 9007199254740993ULL, 0);
    run_bcd_copy("1", 1, 0);
    run_bcd_copy("zero", 0, 0);
    run_bcd_copy("negative zero", 0, 1);

    run_int_to_bcd("123456789012345678", 123456789012345678LL);
    run_int_to_bcd("-123456789012345678", -123456789012345678LL);
    run_int_to_bcd("999999999999999999", 999999999999999999LL);
    run_int_to_bcd("-999999999999999999", -999999999999999999LL);
    run_int_to_bcd("2^53+1", 9007199254740993LL);
    run_int_to_bcd("Currency 922337203685.4775", 9223372036854775LL);
    run_int_to_bcd("-1", -1);
    run_int_to_bcd("zero", 0);
    run_int_to_bcd("10^18 (indefinite)", 1000000000000000000LL);
    run_int_to_bcd("-10^18 (indefinite)", -1000000000000000000LL);
    run_int_to_bcd("INT64_MAX (indefinite)", INT64_MAX);
    run_int_to_bcd("INT64_MIN (indefinite)", INT64_MIN);
}

int main(void) {
    static const struct {
        const char* name;
        uint16_t cw;
    } modes[] = {
        {"RN", CW_NEAREST},
        {"RD", CW_DOWN},
        {"RU", CW_UP},
        {"RZ", CW_TRUNC},
    };
    for (int m = 0; m < 4; m++) {
        printf("-- rounding %s\n", modes[m].name);
        set_cw(modes[m].cw);
        run_all();
    }
    set_cw(CW_NEAREST);

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
