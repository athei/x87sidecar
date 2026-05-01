/*
 * test_fistt.c — Tests for FISTT (FISTTP) direct translation.
 * Verifies truncation-toward-zero semantics for m16, m32, m64 sizes.
 *
 * Build: gcc -O0 -mfpmath=387 -o test_fistt test_fistt.c -lm
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void check_i16(const char* name, int16_t got, int16_t expected) {
    if (got != expected) {
        printf("FAIL  %-52s  got=%d  expected=%d\n", name, (int)got, (int)expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

static void check_i32(const char* name, int32_t got, int32_t expected) {
    if (got != expected) {
        printf("FAIL  %-52s  got=%d  expected=%d\n", name, (int)got, (int)expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

static void check_i64(const char* name, int64_t got, int64_t expected) {
    if (got != expected) {
        printf("FAIL  %-52s  got=%lld  expected=%lld\n", name, (long long)got, (long long)expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

/* === FISTT m16 === */
static int16_t fistt_m16(double v) {
    int16_t r;
    __asm__ volatile("fldl %1\n\t fisttps %0\n" : "=m"(r) : "m"(v));
    return r;
}

/* === FISTT m32 === */
static int32_t fistt_m32(double v) {
    int32_t r;
    __asm__ volatile("fldl %1\n\t fisttpl %0\n" : "=m"(r) : "m"(v));
    return r;
}

/* === FISTT m64 === */
static int64_t fistt_m64(double v) {
    int64_t r;
    __asm__ volatile("fldl %1\n\t fisttpll %0\n" : "=m"(r) : "m"(v));
    return r;
}

int main(void) {
    printf("=== FISTT m16 (truncation toward zero) ===\n");
    check_i16("FISTT m16  0.0", fistt_m16(0.0), 0);
    check_i16("FISTT m16  1.0", fistt_m16(1.0), 1);
    check_i16("FISTT m16  -1.0", fistt_m16(-1.0), -1);
    check_i16("FISTT m16  2.9", fistt_m16(2.9), 2);
    check_i16("FISTT m16  -2.9", fistt_m16(-2.9), -2);
    check_i16("FISTT m16  2.5", fistt_m16(2.5), 2);
    check_i16("FISTT m16  -2.5", fistt_m16(-2.5), -2);
    check_i16("FISTT m16  0.9", fistt_m16(0.9), 0);
    check_i16("FISTT m16  -0.9", fistt_m16(-0.9), 0);
    check_i16("FISTT m16  32767.0", fistt_m16(32767.0), 32767);
    check_i16("FISTT m16  -32768.0", fistt_m16(-32768.0), -32768);
    check_i16("FISTT m16  42.7", fistt_m16(42.7), 42);

    printf("\n=== FISTT m32 (truncation toward zero) ===\n");
    check_i32("FISTT m32  0.0", fistt_m32(0.0), 0);
    check_i32("FISTT m32  1.0", fistt_m32(1.0), 1);
    check_i32("FISTT m32  -1.0", fistt_m32(-1.0), -1);
    check_i32("FISTT m32  2.9", fistt_m32(2.9), 2);
    check_i32("FISTT m32  -2.9", fistt_m32(-2.9), -2);
    check_i32("FISTT m32  2.5", fistt_m32(2.5), 2);
    check_i32("FISTT m32  -2.5", fistt_m32(-2.5), -2);
    check_i32("FISTT m32  0.1", fistt_m32(0.1), 0);
    check_i32("FISTT m32  -0.1", fistt_m32(-0.1), 0);
    check_i32("FISTT m32  100000.99", fistt_m32(100000.99), 100000);
    check_i32("FISTT m32  -100000.99", fistt_m32(-100000.99), -100000);
    check_i32("FISTT m32  2147483647", fistt_m32(2147483647.0), 2147483647);

    printf("\n=== FISTT m64 (truncation toward zero) ===\n");
    check_i64("FISTT m64  0.0", fistt_m64(0.0), 0);
    check_i64("FISTT m64  1.0", fistt_m64(1.0), 1);
    check_i64("FISTT m64  -1.0", fistt_m64(-1.0), -1);
    check_i64("FISTT m64  2.9", fistt_m64(2.9), 2);
    check_i64("FISTT m64  -2.9", fistt_m64(-2.9), -2);
    check_i64("FISTT m64  1e12 + 0.7", fistt_m64(1e12 + 0.7), (int64_t)1000000000000LL);
    check_i64("FISTT m64  -1e12 - 0.7", fistt_m64(-1e12 - 0.7), (int64_t)-1000000000000LL);
    check_i64("FISTT m64  999999.999", fistt_m64(999999.999), 999999);

    printf("\n%s  (%d failure%s)\n", failures == 0 ? "ALL PASS" : "SOME FAILURES", failures,
           failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
