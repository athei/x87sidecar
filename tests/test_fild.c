/*
 * test_fild.c — exercise translate_fild through all three encodings:
 *
 *   DF /0  FILD m16int   (int16_t)
 *   DB /0  FILD m32int   (int32_t)
 *   DF /5  FILD m64int   (int64_t)
 *
 * Build (32-bit x87 baseline, run under Rosetta on Apple Silicon):
 *   gcc -m32 -mfpmath=387 -O0 -o test_fild test_fild.c
 *
 * Build (64-bit, x87 still used for fild):
 *   gcc -O0 -mfpmath=387 -o test_fild test_fild.c
 *
 * Each test:
 *   1. Loads a known integer value via FILD (one of the three encodings).
 *   2. Stores the result from ST(0) to a double with FSTPL.
 *   3. Compares the stored double against the expected IEEE 754 value.
 *
 * Key cases per encoding:
 *   - Zero
 *   - Positive value
 *   - Negative value (exercises sign-extension)
 *   - Minimum representable value (INT16_MIN / INT32_MIN / INT64_MIN)
 *   - Maximum representable value (INT16_MAX / INT32_MAX / INT64_MAX)
 *
 * Stack discipline test: push two values and verify both are correct
 * (catches off-by-one errors in emit_x87_push).
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h> /* memcmp */

/* ------------------------------------------------------------------ */
/* Helpers                                                              */
/* ------------------------------------------------------------------ */

static int failures = 0;

static void check(const char* name, double got, double expected) {
    /* Use memcmp so that -0.0 == +0.0 is treated as unequal if bits differ.
     * For FILD results this shouldn't matter (integers can't produce -0),
     * but it makes the comparison exact. */
    if (memcmp(&got, &expected, sizeof(double)) != 0) {
        /* Also accept bitwise equality via normal == for NaN-free values. */
        if (got != expected) {
            printf("FAIL  %-40s  got=%.20g  expected=%.20g\n", name, got, expected);
            failures++;
            return;
        }
    }
    printf("PASS  %s\n", name);
}

/* ------------------------------------------------------------------ */
/* m16int  — FILD m16int  (DF /0)                                       */
/* ------------------------------------------------------------------ */

static double fild_m16(int16_t v) {
    double result;
    __asm__ volatile(
        "filds %1\n\t"
        "fstpl %0\n\t"
        : "=m"(result)
        : "m"(v)
        : "st");
    return result;
}

static void test_fild_m16(void) {
    check("fild_m16  zero", fild_m16(0), 0.0);
    check("fild_m16  positive", fild_m16(1), 1.0);
    check("fild_m16  42", fild_m16(42), 42.0);
    check("fild_m16  negative -1", fild_m16(-1), -1.0);
    check("fild_m16  negative -42", fild_m16(-42), -42.0);
    check("fild_m16  INT16_MAX", fild_m16(32767), 32767.0);
    check("fild_m16  INT16_MIN", fild_m16(-32768), -32768.0);
    /* Value that would be wrong if zero-extended instead of sign-extended:
     * 0x8000 as unsigned = 32768, as signed int16 = -32768. */
    check("fild_m16  sign-ext 0x8000", fild_m16((int16_t)0x8000), -32768.0);
    /* 0xFF00 as unsigned = 65280, as signed int16 = -256. */
    check("fild_m16  sign-ext 0xFF00", fild_m16((int16_t)0xFF00), -256.0);
}

/* ------------------------------------------------------------------ */
/* m32int  — FILD m32int  (DB /0)                                       */
/* ------------------------------------------------------------------ */

static double fild_m32(int32_t v) {
    double result;
    __asm__ volatile(
        "fildl %1\n\t"
        "fstpl %0\n\t"
        : "=m"(result)
        : "m"(v)
        : "st");
    return result;
}

static void test_fild_m32(void) {
    check("fild_m32  zero", fild_m32(0), 0.0);
    check("fild_m32  positive", fild_m32(1), 1.0);
    check("fild_m32  1000000", fild_m32(1000000), 1000000.0);
    check("fild_m32  negative -1", fild_m32(-1), -1.0);
    check("fild_m32  negative -1M", fild_m32(-1000000), -1000000.0);
    check("fild_m32  INT32_MAX", fild_m32(2147483647), 2147483647.0);
    check("fild_m32  INT32_MIN", fild_m32(-2147483648), -2147483648.0);
    /* 0x80000000 as unsigned = 2147483648, as signed = -2147483648. */
    check("fild_m32  sign-ext 0x80000000", fild_m32((int32_t)0x80000000), -2147483648.0);
    /* 0xFFFF0000 as unsigned = 4294901760, as signed = -65536. */
    check("fild_m32  sign-ext 0xFFFF0000", fild_m32((int32_t)0xFFFF0000), -65536.0);
}

/* ------------------------------------------------------------------ */
/* m64int  — FILD m64int  (DF /5)                                       */
/* ------------------------------------------------------------------ */

static double fild_m64(int64_t v) {
    double result;
    __asm__ volatile(
        "fildll %1\n\t"
        "fstpl %0\n\t"
        : "=m"(result)
        : "m"(v)
        : "st");
    return result;
}

static void test_fild_m64(void) {
    check("fild_m64  zero", fild_m64(0), 0.0);
    check("fild_m64  positive", fild_m64(1), 1.0);
    check("fild_m64  1e12", fild_m64(1000000000000LL), 1e12);
    check("fild_m64  negative -1", fild_m64(-1), -1.0);
    check("fild_m64  negative -1e12", fild_m64(-1000000000000LL), -1e12);
    /* INT64_MAX = 9223372036854775807; not exactly representable in f64,
     * but FILD must produce the nearest double (9223372036854775808.0). */
    check("fild_m64  INT64_MAX", fild_m64(9223372036854775807LL), 9223372036854775808.0);
    /* INT64_MIN = -9223372036854775808; exact in f64. */
    check("fild_m64  INT64_MIN", fild_m64((int64_t)(-9223372036854775807LL - 1)),
          -9223372036854775808.0);
    /* 0x8000000000000000 as signed = INT64_MIN.
     * Would be wrong if the 64-bit load treated it as unsigned. */
    check("fild_m64  sign-ext 0x8000000000000000", fild_m64((int64_t)0x8000000000000000LL),
          -9223372036854775808.0);
    /* A value only representable in 64-bit: larger than INT32_MAX. */
    check("fild_m64  2^33", fild_m64(8589934592LL), 8589934592.0);
}

/* ------------------------------------------------------------------ */
/* Stack discipline — two pushes, verify both slots                     */
/* ------------------------------------------------------------------ */

static void test_fild_stack(void) {
    /* Push two values with different encodings, then pop both and verify.
     * This exercises emit_x87_push twice and checks that TOP is updated
     * correctly between the two FILDs. */
    int16_t v16 = -7;
    int32_t v32 = 100;
    double r0, r1;

    __asm__ volatile(
        /* push v32 first: ST(0) = 100 */
        "fildl %2\n\t"
        /* push v16 second: ST(0) = -7, ST(1) = 100 */
        "filds %3\n\t"
        /* pop ST(0) → r0, then ST(0) → r1 */
        "fstpl %0\n\t"
        "fstpl %1\n\t"
        : "=m"(r0), "=m"(r1)
        : "m"(v32), "m"(v16)
        : "st");

    check("fild_stack  top  (m16 -7)", r0, -7.0);
    check("fild_stack  next (m32 100)", r1, 100.0);

    /* Same again with m64 on top */
    int64_t v64 = 999999999999LL;
    int32_t v32b = -1;
    double r2, r3;

    __asm__ volatile(
        "fildl %2\n\t"
        "fildll %3\n\t"
        "fstpl %0\n\t"
        "fstpl %1\n\t"
        : "=m"(r2), "=m"(r3)
        : "m"(v32b), "m"(v64)
        : "st");

    check("fild_stack  top  (m64 999999999999)", r2, 999999999999.0);
    check("fild_stack  next (m32 -1)", r3, -1.0);
}

/* ------------------------------------------------------------------ */
/* fild m64 values that must not be copied raw                          */
/* ------------------------------------------------------------------ */

/* The sidecar copies fild m64 into fistp m64 as raw bits.  These shapes
 * look similar but use the value as a number, so they must keep converting. */

static void check_i64(const char* name, int64_t got, int64_t expected) {
    if (got != expected) {
        printf("FAIL  %-40s  got=0x%016llx  expected=0x%016llx\n", name, (unsigned long long)got,
               (unsigned long long)expected);
        failures++;
        return;
    }
    printf("PASS  %s\n", name);
}

static void check_i32(const char* name, int32_t got, int32_t expected) {
    if (got != expected) {
        printf("FAIL  %-40s  got=0x%08x  expected=0x%08x\n", name, (uint32_t)got,
               (uint32_t)expected);
        failures++;
        return;
    }
    printf("PASS  %s\n", name);
}

/* Accept either result: real x87 keeps 64 mantissa bits, the sidecar's
 * binary64 rounds.  The integer indefinite is wrong under both. */
static void check_i64_either(const char* name, int64_t got, int64_t exact, int64_t rounded) {
    if (got == (int64_t)0x8000000000000000ULL || (got != exact && got != rounded)) {
        printf("FAIL  %-40s  got=0x%016llx  expected=0x%016llx or 0x%016llx\n", name,
               (unsigned long long)got, (unsigned long long)exact, (unsigned long long)rounded);
        failures++;
        return;
    }
    printf("PASS  %s\n", name);
}

static void test_fild_m64_not_raw(void) {
    /* Arithmetic in between: 2^52 + 1 + 1 is exact in binary64 too, and a
     * raw copy would store 2^52 + 1. */
    {
        const int64_t v = (1LL << 52) + 1;
        int64_t out = 0;
        __asm__ volatile(
            "fildll  %1\n\t"
            "fld1\n\t"
            "faddp\n\t"
            "fistpll %0\n\t"
            : "=m"(out)
            : "m"(v)
            : "st");
        check_i64("fildll fld1 faddp fistpll", out, 0x0010000000000002LL);
    }

    /* Narrower store: the value converts, it is not truncated to its low
     * 32 bits. */
    {
        const int64_t v = 123456789;
        int32_t out = 0;
        __asm__ volatile(
            "fildll  %1\n\t"
            "fistpl  %0\n\t"
            : "=m"(out)
            : "m"(v)
            : "st");
        check_i32("fildll fistpl in range", out, 123456789);
    }
    {
        const int64_t v = 0x5357452E46535745LL;
        int32_t out = 0;
        __asm__ volatile(
            "fildll  %1\n\t"
            "fistpl  %0\n\t"
            : "=m"(out)
            : "m"(v)
            : "st");
        check_i32("fildll fistpl out of range", out, (int32_t)0x80000000);
    }

    /* The same value stored as m32 and then m64: the m64 store is not a
     * pure copy, so the sidecar still rounds it through binary64. */
    {
        const int64_t v = 0x5357452E46535745LL;
        int32_t out32 = 0;
        int64_t out64 = 0;
        __asm__ volatile(
            "fildll  %2\n\t"
            "fistl   %0\n\t"
            "fistpll %1\n\t"
            : "=m"(out32), "=m"(out64)
            : "m"(v)
            : "st");
        check_i32("fildll fistl fistpll, m32", out32, (int32_t)0x80000000);
        check_i64_either("fildll fistl fistpll, m64", out64, v, 0x5357452E46535800LL);
    }
}

/* ------------------------------------------------------------------ */
/* Entry point                                                          */
/* ------------------------------------------------------------------ */

int main(void) {
    printf("=== FILD m16int (DF /0) ===\n");
    test_fild_m16();

    printf("\n=== FILD m32int (DB /0) ===\n");
    test_fild_m32();

    printf("\n=== FILD m64int (DF /5) ===\n");
    test_fild_m64();

    printf("\n=== Stack discipline ===\n");
    test_fild_stack();

    printf("\n=== FILD m64 used as a number ===\n");
    test_fild_m64_not_raw();

    printf("\n%s  (%d failure%s)\n", failures == 0 ? "ALL PASS" : "SOME FAILURES", failures,
           failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}