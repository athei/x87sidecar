/*
 * test_fist_indefinite.c — x86 integer-indefinite semantics for FIST /
 * FISTP / FISTTP.
 *
 * x86 stores the destination width's INT_MIN (0x8000 / 0x80000000 /
 * 0x8000000000000000, the "integer indefinite") when the source is NaN or
 * the rounded value does not fit the destination — for BOTH overflow
 * directions.  AArch64 FCVT*S saturates (+overflow → INT_MAX) and converts
 * NaN to 0, so the sidecar must patch the result explicitly.  The range
 * check happens after rounding per RC: 32767.5 fits m16 under truncate but
 * not under round-to-nearest.
 *
 * Covers the direct translators (single op per run), the IR-run path
 * (several stores per run, incl. the cached-RC dispatch), and FISTTP.
 *
 * Build: clang -arch x86_64 -O0 -o test_fist_indefinite test_fist_indefinite.c
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static int failures = 0;

#define INDEF16 ((int16_t)0x8000)
#define INDEF32 ((int32_t)0x80000000)
#define INDEF64 ((int64_t)0x8000000000000000ULL)

static void check_i16(const char* name, int16_t got, int16_t expected) {
    if (got != expected) {
        printf("FAIL  %-52s  got=0x%04x  expected=0x%04x\n", name, (uint16_t)got,
               (uint16_t)expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

static void check_i32(const char* name, int32_t got, int32_t expected) {
    if (got != expected) {
        printf("FAIL  %-52s  got=0x%08x  expected=0x%08x\n", name, (uint32_t)got,
               (uint32_t)expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

static void check_i64(const char* name, int64_t got, int64_t expected) {
    if (got != expected) {
        printf("FAIL  %-52s  got=0x%016llx  expected=0x%016llx\n", name, (unsigned long long)got,
               (unsigned long long)expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

/* ── Rounding mode helpers ────────────────────────────────────────────────── */

#define CW_NEAREST 0x037F
#define CW_FLOOR 0x077F
#define CW_CEIL 0x0B7F
#define CW_TRUNC 0x0F7F

static void set_cw(uint16_t cw) {
    __asm__ volatile("fldcw %0" : : "m"(cw));
}

/* ── Single-op wrappers (direct translator path) ──────────────────────────── */

static int16_t fistp_i16(double x) {
    int16_t r;
    __asm__ volatile("fldl %1\n\tfistps %0\n" : "=m"(r) : "m"(x));
    return r;
}

static int32_t fistp_i32(double x) {
    int32_t r;
    __asm__ volatile("fldl %1\n\tfistpl %0\n" : "=m"(r) : "m"(x));
    return r;
}

static int64_t fistp_i64(double x) {
    int64_t r;
    __asm__ volatile("fldl %1\n\tfistpll %0\n" : "=m"(r) : "m"(x));
    return r;
}

static int16_t fisttp_i16(double x) {
    int16_t r;
    __asm__ volatile("fldl %1\n\tfisttps %0\n" : "=m"(r) : "m"(x));
    return r;
}

static int32_t fisttp_i32(double x) {
    int32_t r;
    __asm__ volatile("fldl %1\n\tfisttpl %0\n" : "=m"(r) : "m"(x));
    return r;
}

static int64_t fisttp_i64(double x) {
    int64_t r;
    __asm__ volatile("fldl %1\n\tfisttpll %0\n" : "=m"(r) : "m"(x));
    return r;
}

/* FIST (non-popping): store twice, pop once — both stores must agree. */
static void fist_fistp_i32(double x, int32_t* r1, int32_t* r2) {
    __asm__ volatile(
        "fldl   %2\n\t"
        "fistl  %0\n\t"
        "fistpl %1\n"
        : "=m"(*r1), "=m"(*r2)
        : "m"(x));
}

/* ── Multi-store single-run wrappers (IR path, cached-RC dispatch) ────────── */

static void fistp_i32_x2(double a, double b, int32_t* ra, int32_t* rb) {
    __asm__ volatile(
        "fldl  %2\n\t"
        "fldl  %3\n\t"
        "fistpl %0\n\t"
        "fistpl %1\n"
        : "=m"(*ra), "=m"(*rb)
        : "m"(b), "m"(a));
}

static void fistp_mixed_sizes(double a, double b, double c, int16_t* ra, int32_t* rb, int64_t* rc) {
    __asm__ volatile(
        "fldl  %3\n\t"
        "fldl  %4\n\t"
        "fldl  %5\n\t"
        "fistps %0\n\t"
        "fistpl %1\n\t"
        "fistpll %2\n"
        : "=m"(*ra), "=m"(*rb), "=m"(*rc)
        : "m"(c), "m"(b), "m"(a));
}

int main(void) {
    const double qnan = NAN;
    /* Largest double strictly below 2^63: 2^63 - 1024. */
    const double below_2p63 = 9223372036854774784.0;

    set_cw(CW_NEAREST);

    /* ── m16 ─────────────────────────────────────────────────────────────── */
    check_i16("fistp i16  NaN → indefinite", fistp_i16(qnan), INDEF16);
    check_i16("fistp i16  40000.0 → indefinite (no wrap)", fistp_i16(40000.0), INDEF16);
    check_i16("fistp i16  -32769.0 → indefinite", fistp_i16(-32769.0), INDEF16);
    check_i16("fistp i16  1e30 → indefinite", fistp_i16(1e30), INDEF16);
    check_i16("fistp i16  32767.0 in range", fistp_i16(32767.0), 32767);
    check_i16("fistp i16  -32768.0 in range", fistp_i16(-32768.0), INDEF16 /* == -32768 */);
    check_i16("fistp i16  12.75 → 13 (RN)", fistp_i16(12.75), 13);

    /* ── m32 ─────────────────────────────────────────────────────────────── */
    check_i32("fistp i32  NaN → indefinite", fistp_i32(qnan), INDEF32);
    check_i32("fistp i32  2^40 → indefinite (not INT_MAX)", fistp_i32(1099511627776.0), INDEF32);
    check_i32("fistp i32  -2^40 → indefinite", fistp_i32(-1099511627776.0), INDEF32);
    check_i32("fistp i32  2147483647.0 in range", fistp_i32(2147483647.0), 2147483647);
    check_i32("fistp i32  -2147483648.0 in range", fistp_i32(-2147483648.0), INT32_MIN);

    /* ── m64 ─────────────────────────────────────────────────────────────── */
    check_i64("fistp i64  NaN → indefinite", fistp_i64(qnan), INDEF64);
    check_i64("fistp i64  2^63 → indefinite", fistp_i64(9223372036854775808.0), INDEF64);
    check_i64("fistp i64  1e30 → indefinite", fistp_i64(1e30), INDEF64);
    check_i64("fistp i64  -1e30 → indefinite", fistp_i64(-1e30), INDEF64);
    check_i64("fistp i64  -2^63 in range (== indefinite bits)", fistp_i64(-9223372036854775808.0),
              INT64_MIN);
    check_i64("fistp i64  2^63-1024 in range", fistp_i64(below_2p63), 9223372036854774784LL);
    check_i64("fistp i64  2^62 in range", fistp_i64(4611686018427387904.0), 4611686018427387904LL);

    /* ── Post-rounding boundary: fits under some RC modes only ───────────── */
    set_cw(CW_NEAREST); /* 32767.5 → 32768 (ties-to-even) → overflow */
    check_i16("fistp i16  32767.5 RN → indefinite", fistp_i16(32767.5), INDEF16);
    set_cw(CW_TRUNC); /* → 32767 fits */
    check_i16("fistp i16  32767.5 RZ → 32767", fistp_i16(32767.5), 32767);
    set_cw(CW_FLOOR); /* → 32767 fits */
    check_i16("fistp i16  32767.5 RD → 32767", fistp_i16(32767.5), 32767);
    set_cw(CW_CEIL); /* → 32768 → overflow */
    check_i16("fistp i16  32767.5 RU → indefinite", fistp_i16(32767.5), INDEF16);

    set_cw(CW_NEAREST); /* 2147483647.5 → 2147483648 (ties-to-even) → overflow */
    check_i32("fistp i32  2147483647.5 RN → indefinite", fistp_i32(2147483647.5), INDEF32);
    set_cw(CW_TRUNC); /* → 2147483647 fits */
    check_i32("fistp i32  2147483647.5 RZ → INT_MAX", fistp_i32(2147483647.5), 2147483647);
    set_cw(CW_FLOOR); /* -2147483648.5 → -2147483649 → overflow */
    check_i32("fistp i32  -2147483648.5 RD → indefinite", fistp_i32(-2147483648.5), INDEF32);
    set_cw(CW_TRUNC); /* → -2147483648 fits */
    check_i32("fistp i32  -2147483648.5 RZ → INT_MIN", fistp_i32(-2147483648.5), INT32_MIN);

    set_cw(CW_NEAREST);

    /* ── FISTTP (truncating, ignores RC) ─────────────────────────────────── */
    check_i16("fisttp i16  NaN → indefinite", fisttp_i16(qnan), INDEF16);
    check_i16("fisttp i16  40000.0 → indefinite", fisttp_i16(40000.0), INDEF16);
    check_i16("fisttp i16  32767.9 → 32767", fisttp_i16(32767.9), 32767);
    check_i32("fisttp i32  NaN → indefinite", fisttp_i32(qnan), INDEF32);
    check_i32("fisttp i32  2^40 → indefinite", fisttp_i32(1099511627776.0), INDEF32);
    check_i32("fisttp i32  2147483647.5 → INT_MAX (trunc)", fisttp_i32(2147483647.5), 2147483647);
    check_i64("fisttp i64  NaN → indefinite", fisttp_i64(qnan), INDEF64);
    check_i64("fisttp i64  2^63 → indefinite", fisttp_i64(9223372036854775808.0), INDEF64);
    check_i64("fisttp i64  2^63-1024 in range", fisttp_i64(below_2p63), 9223372036854774784LL);

    /* ── FIST (non-popping) ──────────────────────────────────────────────── */
    {
        int32_t r1 = 1, r2 = 2;
        fist_fistp_i32(qnan, &r1, &r2);
        check_i32("fist i32  NaN → indefinite", r1, INDEF32);
        check_i32("fistp after fist  NaN → indefinite", r2, INDEF32);
        fist_fistp_i32(1099511627776.0, &r1, &r2);
        check_i32("fist i32  2^40 → indefinite", r1, INDEF32);
        check_i32("fistp after fist  2^40 → indefinite", r2, INDEF32);
    }

    /* ── Multi-store IR runs (cached-RC dispatch path) ───────────────────── */
    {
        int32_t ra = 0, rb = 0;
        fistp_i32_x2(qnan, 1099511627776.0, &ra, &rb);
        check_i32("x2 run  NaN → indefinite", ra, INDEF32);
        check_i32("x2 run  2^40 → indefinite", rb, INDEF32);

        fistp_i32_x2(-7.5, 1e30, &ra, &rb);
        check_i32("x2 run  -7.5 → -8 (RN, in range)", ra, -8);
        check_i32("x2 run  1e30 → indefinite", rb, INDEF32);
    }
    {
        int16_t ra = 0;
        int32_t rb = 0;
        int64_t rc = 0;
        fistp_mixed_sizes(40000.0, qnan, 1e30, &ra, &rb, &rc);
        check_i16("mixed run  40000.0 → i16 indefinite", ra, INDEF16);
        check_i32("mixed run  NaN → i32 indefinite", rb, INDEF32);
        check_i64("mixed run  1e30 → i64 indefinite", rc, INDEF64);

        fistp_mixed_sizes(-129.0, 70000.0, below_2p63, &ra, &rb, &rc);
        check_i16("mixed run  -129.0 → i16 in range", ra, -129);
        check_i32("mixed run  70000.0 → i32 in range", rb, 70000);
        check_i64("mixed run  2^63-1024 → i64 in range", rc, 9223372036854774784LL);
    }

    printf("%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
