/*
 * test_fistp_multi.c — Tests for multiple FISTP/FIST in a single IR run.
 *
 * Verifies that rounding-mode caching (hoisting the control_word LDRH+UBFX)
 * produces correct results when 2+ non-truncating integer stores share
 * the same IR run.  FLDCW bails the IR, so all FISTPs within a run see
 * the same RC — this test confirms the cached dispatch agrees with the
 * single-FISTP path for all four rounding modes.
 *
 * Build: clang -arch x86_64 -O0 -o test_fistp_multi test_fistp_multi.c
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void check_i32(const char* name, int32_t got, int32_t expected) {
    if (got != expected) {
        printf("FAIL  %-52s  got=%d  expected=%d\n", name, (int)got, (int)expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

static void check_i16(const char* name, int16_t got, int16_t expected) {
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

/* ── 2x FISTP m32 in one IR run ───────────────────────────────────────────── */

static void fistp_m32_x2(double a, double b, int32_t* ra, int32_t* rb) {
    __asm__ volatile(
        "fldl  %2\n\t"  /* push b  → ST(0)=b                */
        "fldl  %3\n\t"  /* push a  → ST(0)=a, ST(1)=b       */
        "fistpl %0\n\t" /* pop ST(0)=a → *ra, ST(0)=b       */
        "fistpl %1\n"   /* pop ST(0)=b → *rb                 */
        : "=m"(*ra), "=m"(*rb)
        : "m"(b), "m"(a));
}

/* ── 3x FISTP m32 in one IR run ───────────────────────────────────────────── */

static void fistp_m32_x3(double a, double b, double c, int32_t* ra, int32_t* rb, int32_t* rc) {
    __asm__ volatile(
        "fldl  %3\n\t"  /* push c  → ST(0)=c                          */
        "fldl  %4\n\t"  /* push b  → ST(0)=b, ST(1)=c                 */
        "fldl  %5\n\t"  /* push a  → ST(0)=a, ST(1)=b, ST(2)=c       */
        "fistpl %0\n\t" /* pop a → *ra                                 */
        "fistpl %1\n\t" /* pop b → *rb                                 */
        "fistpl %2\n"   /* pop c → *rc                                 */
        : "=m"(*ra), "=m"(*rb), "=m"(*rc)
        : "m"(c), "m"(b), "m"(a));
}

/* ── 4x FISTP m32 in one IR run ───────────────────────────────────────────── */

static void fistp_m32_x4(double a, double b, double c, double d, int32_t* ra, int32_t* rb,
                         int32_t* rc, int32_t* rd) {
    __asm__ volatile(
        "fldl  %4\n\t"
        "fldl  %5\n\t"
        "fldl  %6\n\t"
        "fldl  %7\n\t"
        "fistpl %0\n\t"
        "fistpl %1\n\t"
        "fistpl %2\n\t"
        "fistpl %3\n"
        : "=m"(*ra), "=m"(*rb), "=m"(*rc), "=m"(*rd)
        : "m"(d), "m"(c), "m"(b), "m"(a));
}

/* ── Mixed sizes: FISTP m16 + FISTP m32 + FISTP m64 ──────────────────────── */

static void fistp_mixed_sizes(double a, double b, double c, int16_t* ra, int32_t* rb, int64_t* rc) {
    __asm__ volatile(
        "fldl  %3\n\t"  /* push c */
        "fldl  %4\n\t"  /* push b */
        "fldl  %5\n\t"  /* push a */
        "fistps %0\n\t" /* pop a → i16 *ra */
        "fistpl %1\n\t" /* pop b → i32 *rb */
        "fistpll %2\n"  /* pop c → i64 *rc */
        : "=m"(*ra), "=m"(*rb), "=m"(*rc)
        : "m"(c), "m"(b), "m"(a));
}

/* ── FIST (non-popping) x2 + cleanup ─────────────────────────────────────── */

static void fist_m32_x2(double a, int32_t* r1, int32_t* r2) {
    __asm__ volatile(
        "fldl  %2\n\t"    /* push a */
        "fistl %0\n\t"    /* store ST(0)=a → *r1, no pop */
        "fistl %1\n\t"    /* store ST(0)=a → *r2, no pop */
        "fstp  %%st(0)\n" /* clean up */
        : "=m"(*r1), "=m"(*r2)
        : "m"(a));
}

/* ── FISTP + FISTTP mix (only FISTPs use RC dispatch) ─────────────────────── */

static void fistp_fisttp_mix(double a, double b, int32_t* ra, int32_t* rb) {
    __asm__ volatile(
        "fldl   %2\n\t"  /* push b */
        "fldl   %3\n\t"  /* push a */
        "fistpl  %0\n\t" /* pop a → *ra  (RC dispatch) */
        "fisttpl %1\n"   /* pop b → *rb  (always truncate) */
        : "=m"(*ra), "=m"(*rb)
        : "m"(b), "m"(a));
}

/* ── Rounding mode helpers ────────────────────────────────────────────────── */

#define CW_NEAREST 0x037F
#define CW_FLOOR 0x077F
#define CW_CEIL 0x0B7F
#define CW_TRUNC 0x0F7F

static void set_cw(uint16_t cw) {
    __asm__ volatile("fldcw %0" : : "m"(cw));
}

static uint16_t get_cw(void) {
    uint16_t cw;
    __asm__ volatile("fnstcw %0" : "=m"(cw));
    return cw;
}

/* ── 2x FISTP with explicit rounding mode ─────────────────────────────────── */

static void fistp_m32_x2_with_rc(double a, double b, int32_t* ra, int32_t* rb, uint16_t cw) {
    /* Set RC before the IR run; FLDCW is outside the run. */
    set_cw(cw);
    fistp_m32_x2(a, b, ra, rb);
}

int main(void) {
    uint16_t saved_cw = get_cw();

    /* ── 2x FISTP m32 (default RC = nearest) ──────────────────────────────── */
    printf("=== 2x FISTP m32 (round-to-nearest) ===\n");
    {
        int32_t ra, rb;
        fistp_m32_x2(1.5, 2.5, &ra, &rb);
        check_i32("2x FISTP m32  1.5 → nearest", ra, 2); /* ties to even */
        check_i32("2x FISTP m32  2.5 → nearest", rb, 2);

        fistp_m32_x2(3.7, -3.7, &ra, &rb);
        check_i32("2x FISTP m32  3.7 → nearest", ra, 4);
        check_i32("2x FISTP m32 -3.7 → nearest", rb, -4);

        fistp_m32_x2(0.0, -0.0, &ra, &rb);
        check_i32("2x FISTP m32  0.0 → nearest", ra, 0);
        check_i32("2x FISTP m32 -0.0 → nearest", rb, 0);
    }

    /* ── 3x FISTP m32 ────────────────────────────────────────────────────── */
    printf("\n=== 3x FISTP m32 (round-to-nearest) ===\n");
    {
        int32_t ra, rb, rc;
        fistp_m32_x3(10.3, 20.7, 30.5, &ra, &rb, &rc);
        check_i32("3x FISTP m32  10.3", ra, 10);
        check_i32("3x FISTP m32  20.7", rb, 21);
        check_i32("3x FISTP m32  30.5", rc, 30); /* ties to even */

        fistp_m32_x3(-1.1, -2.9, -3.5, &ra, &rb, &rc);
        check_i32("3x FISTP m32 -1.1", ra, -1);
        check_i32("3x FISTP m32 -2.9", rb, -3);
        check_i32("3x FISTP m32 -3.5", rc, -4); /* ties to even */
    }

    /* ── 4x FISTP m32 ────────────────────────────────────────────────────── */
    printf("\n=== 4x FISTP m32 (round-to-nearest) ===\n");
    {
        int32_t ra, rb, rc, rd;
        fistp_m32_x4(100.1, 200.9, 300.5, 400.4, &ra, &rb, &rc, &rd);
        check_i32("4x FISTP m32  100.1", ra, 100);
        check_i32("4x FISTP m32  200.9", rb, 201);
        check_i32("4x FISTP m32  300.5", rc, 300); /* ties to even */
        check_i32("4x FISTP m32  400.4", rd, 400);
    }

    /* ── Mixed sizes ──────────────────────────────────────────────────────── */
    printf("\n=== Mixed sizes: FISTP m16 + m32 + m64 ===\n");
    {
        int16_t ra;
        int32_t rb;
        int64_t rc;
        fistp_mixed_sizes(42.7, 100000.3, 1e12 + 0.7, &ra, &rb, &rc);
        check_i16("mixed  42.7 → i16", ra, 43);
        check_i32("mixed  100000.3 → i32", rb, 100000);
        check_i64("mixed  1e12+0.7 → i64", rc, (int64_t)1000000000001LL);
    }

    /* ── FIST (non-popping) x2 ────────────────────────────────────────────── */
    printf("\n=== 2x FIST m32 (non-popping) ===\n");
    {
        int32_t r1, r2;
        fist_m32_x2(7.5, &r1, &r2);
        check_i32("FIST m32 #1  7.5 → nearest", r1, 8); /* ties to even */
        check_i32("FIST m32 #2  7.5 → nearest", r2, 8);

        fist_m32_x2(-2.3, &r1, &r2);
        check_i32("FIST m32 #1 -2.3 → nearest", r1, -2);
        check_i32("FIST m32 #2 -2.3 → nearest", r2, -2);
    }

    /* ── FISTP + FISTTP mix ───────────────────────────────────────────────── */
    printf("\n=== FISTP + FISTTP mix ===\n");
    {
        int32_t ra, rb;
        fistp_fisttp_mix(2.9, 2.9, &ra, &rb);
        check_i32("FISTP  2.9 → nearest (3)", ra, 3);
        check_i32("FISTTP 2.9 → truncate (2)", rb, 2);

        fistp_fisttp_mix(-2.9, -2.9, &ra, &rb);
        check_i32("FISTP  -2.9 → nearest (-3)", ra, -3);
        check_i32("FISTTP -2.9 → truncate (-2)", rb, -2);
    }

    /* ── 2x FISTP with floor rounding ─────────────────────────────────────── */
    printf("\n=== 2x FISTP m32 (floor) ===\n");
    {
        int32_t ra, rb;
        fistp_m32_x2_with_rc(2.9, -2.9, &ra, &rb, CW_FLOOR);
        check_i32("floor  2.9", ra, 2);
        check_i32("floor -2.9", rb, -3);

        fistp_m32_x2_with_rc(0.1, -0.1, &ra, &rb, CW_FLOOR);
        check_i32("floor  0.1", ra, 0);
        check_i32("floor -0.1", rb, -1);
    }

    /* ── 2x FISTP with ceil rounding ──────────────────────────────────────── */
    printf("\n=== 2x FISTP m32 (ceil) ===\n");
    {
        int32_t ra, rb;
        fistp_m32_x2_with_rc(2.1, -2.1, &ra, &rb, CW_CEIL);
        check_i32("ceil   2.1", ra, 3);
        check_i32("ceil  -2.1", rb, -2);

        fistp_m32_x2_with_rc(0.9, -0.9, &ra, &rb, CW_CEIL);
        check_i32("ceil   0.9", ra, 1);
        check_i32("ceil  -0.9", rb, 0);
    }

    /* ── 2x FISTP with truncation rounding ────────────────────────────────── */
    printf("\n=== 2x FISTP m32 (truncate) ===\n");
    {
        int32_t ra, rb;
        fistp_m32_x2_with_rc(2.9, -2.9, &ra, &rb, CW_TRUNC);
        check_i32("trunc  2.9", ra, 2);
        check_i32("trunc -2.9", rb, -2);

        fistp_m32_x2_with_rc(0.9, -0.9, &ra, &rb, CW_TRUNC);
        check_i32("trunc  0.9", ra, 0);
        check_i32("trunc -0.9", rb, 0);
    }

    /* Restore original control word */
    set_cw(saved_cw);

    printf("\n%s  (%d failure%s)\n", failures == 0 ? "ALL PASS" : "SOME FAILURES", failures,
           failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
