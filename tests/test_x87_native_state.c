/* Native save areas must contain real f80 values, including across x87 runs. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures;

static void test_boundary_bits(void) {
    const uint64_t cases[] = {
        0,
        1,
        2,
        0x000fffffffffffff,
        0x0010000000000000,
        0x8000000000000001,
        0x800fffffffffffff,
        0x8000000000000000,
        0x3ff0000000000000,
        0x7fefffffffffffff,
        0x7ff0000000000000,
        0xfff0000000000000,
        0x7ff8000000000001,
        0xffffffffffffffff,
    };
    int bad = 0;
    uint64_t random = 0x123456789abcdef;
    for (unsigned i = 0; i < 4096; ++i) {
        random ^= random << 13;
        random ^= random >> 7;
        random ^= random << 17;
        uint64_t bits = i < sizeof(cases) / sizeof(cases[0]) ? cases[i] : random;
        /* Quiet NaNs: a load may quiet a signaling NaN architecturally. */
        if ((bits & UINT64_C(0x7fffffffffffffff)) > UINT64_C(0x7ff0000000000000))
            bits |= UINT64_C(0x0008000000000000);
        uint64_t got;
        __asm__ volatile("fldl %1; jmp 1f; 1: fstpl %0" : "=m"(got) : "m"(bits) : "st");
        if (got != bits) {
            if (bad++ < 5)
                printf("FAIL  boundary bits in=%016llx out=%016llx\n", bits, got);
        }
    }
    printf("%s  native boundary: 4096 binary64 bit patterns\n", bad ? "FAIL" : "PASS");
    failures += bad != 0;
}

static void test_extended_rounding(void) {
    const struct {
        uint64_t mant;
        uint16_t exp;
        uint64_t expected;
    } cases[] = {
        {0x8000000000000400, 0x3fff, 0x3ff0000000000000},  // even tie
        {0x8000000000000c00, 0x3fff, 0x3ff0000000000002},  // odd tie
        {0x8000000000000000, 0x3bcc, 0},                   // half minimum subnormal
        {0x8000000000000001, 0x3bcc, 1},
        {0x8000000000000000, 0xbbcc, 0x8000000000000000},
        {0xffffffffffffffff, 0x43fe, 0x7ff0000000000000},  // rounded overflow
        {0x8000000000000000, 0x43ff, 0x7ff0000000000000},  // exponent overflow
        {0x8000000000000001, 0x7fff, 0x7ff8000000000000},  // low NaN payload
    };
    int bad = 0;
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        unsigned char native[10];
        memcpy(native, &cases[i].mant, 8);
        memcpy(native + 8, &cases[i].exp, 2);
        uint64_t got;
        __asm__ volatile("fldt %1; jmp 1f; 1: fstpl %0" : "=m"(got) : "m"(native) : "st");
        if (got != cases[i].expected) {
            printf("FAIL  extended rounding case=%u got=%016llx expected=%016llx\n", i, got,
                   cases[i].expected);
            bad++;
        }
    }
    printf("%s  extended rounding: ties, underflow, overflow and NaN\n", bad ? "FAIL" : "PASS");
    failures += bad != 0;
}

static void test_fxsave_payload(void) {
    unsigned char state[512] __attribute__((aligned(16)));
    const double values[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    double restored[8];
    __asm__ volatile(
        "fninit\n"
        "fldl 0(%1); fldl 8(%1); fldl 16(%1); fldl 24(%1)\n"
        "fldl 32(%1); fldl 40(%1); fldl 48(%1); fldl 56(%1)\n"
        "fxsave %0\nfninit"
        : "=m"(state)
        : "r"(values)
        : "memory", "st", "st(1)", "st(2)", "st(3)", "st(4)", "st(5)", "st(6)", "st(7)");
    int bad = 0;
    for (unsigned i = 0; i < 8; ++i) {
        long double expected = values[7 - i];
        if (memcmp(state + 32 + 16 * i, &expected, 10)) {
            printf("FAIL  fxsave native payload slot=%u\n", i);
            bad++;
        }
        /* Edit every slot to test native input independently of our output. */
        long double replacement = values[i] + 0.25L;
        memcpy(state + 32 + 16 * i, &replacement, 10);
    }
    __asm__ volatile(
        "fxrstor %1\n"
        "fstpl 0(%0); fstpl 8(%0); fstpl 16(%0); fstpl 24(%0)\n"
        "fstpl 32(%0); fstpl 40(%0); fstpl 48(%0); fstpl 56(%0)"
        :
        : "r"(restored), "m"(state)
        : "memory", "st", "st(1)", "st(2)", "st(3)", "st(4)", "st(5)", "st(6)", "st(7)");
    for (unsigned i = 0; i < 8; ++i)
        if (restored[i] != values[i] + 0.25) {
            printf("FAIL  fxrstor native payload slot=%u got=%g\n", i, restored[i]);
            bad++;
        }
    printf("%s  native save/restore: all eight slots\n", bad ? "FAIL" : "PASS");
    failures += bad != 0;
}

int main(void) {
    test_boundary_bits();
    test_extended_rounding();
    test_fxsave_payload();
    return failures ? 1 : 0;
}
