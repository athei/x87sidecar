/*
 * test_frndint.c — Tests for FRNDINT in the IR pipeline.
 *
 * FRNDINT rounds ST(0) to an integer using the current rounding mode (RC).
 * Games use FLDCW + FRNDINT for floor()/ceil() patterns.
 *
 * Key scenarios:
 *   1. FRNDINT standalone with each RC mode (nearest, floor, ceil, truncate).
 *   2. FLDCW + FRNDINT in one run (the common game pattern).
 *   3. FRNDINT + FSTP in one run (result stored back to memory).
 *   4. FLD + FRNDINT + FLD + FADD + FSTP (FRNDINT mid-arithmetic).
 *   5. Multiple FRNDINTs in a single run.
 *
 * Build: clang -arch x86_64 -O0 -o test_frndint test_frndint.c
 */
#include <stdint.h>
#include <stdio.h>

static int failures = 0;

static void check_f64(const char* name, double got, double expected) {
    if (got != expected) {
        printf("FAIL  %-55s  got=%g  expected=%g\n", name, got, expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

#define CW_NEAREST 0x037F /* RC=00 */
#define CW_FLOOR 0x077F   /* RC=01 */
#define CW_CEIL 0x0B7F    /* RC=10 */
#define CW_TRUNC 0x0F7F   /* RC=11 */

static void set_cw(uint16_t cw) {
    __asm__ volatile("fldcw %0" : : "m"(cw));
}

static uint16_t get_cw(void) {
    uint16_t cw;
    __asm__ volatile("fnstcw %0" : "=m"(cw));
    return cw;
}

/* ── 1. Standalone FRNDINT (RC set before the run) ────────────────────────
 * FLD val + FRNDINT + FSTP result  (3-insn run).
 */
static double frndint_standalone(double val) {
    double result;
    __asm__ volatile(
        "fldl   %1\n\t"
        "frndint\n\t"
        "fstpl  %0\n"
        : "=m"(result)
        : "m"(val));
    return result;
}

/* ── 2. FLDCW + FRNDINT in one run (the game floor/ceil pattern) ─────────
 * FLDCW + FLD val + FRNDINT + FSTP result  (4-insn run).
 */
static double fldcw_frndint(double val, uint16_t cw) {
    double result;
    __asm__ volatile(
        "fldcw  %2\n\t"
        "fldl   %1\n\t"
        "frndint\n\t"
        "fstpl  %0\n"
        : "=m"(result)
        : "m"(val), "m"(cw));
    return result;
}

/* ── 3. FRNDINT mid-arithmetic ───────────────────────────────────────────
 * FLD a + FRNDINT + FLD b + FADDP + FSTP  (5-insn run).
 * Rounds a, then adds b (unrounded).
 */
static double frndint_then_add(double a, double b) {
    double result;
    __asm__ volatile(
        "fldl   %1\n\t"
        "frndint\n\t"
        "fldl   %2\n\t"
        "faddp\n\t"
        "fstpl  %0\n"
        : "=m"(result)
        : "m"(a), "m"(b));
    return result;
}

/* ── 4. Double FRNDINT in one run ────────────────────────────────────────
 * FLD a + FRNDINT + FLD b + FRNDINT + FADDP + FSTP  (6-insn run).
 * Rounds both a and b independently, then adds.
 */
static double frndint_double(double a, double b) {
    double result;
    __asm__ volatile(
        "fldl   %1\n\t"
        "frndint\n\t"
        "fldl   %2\n\t"
        "frndint\n\t"
        "faddp\n\t"
        "fstpl  %0\n"
        : "=m"(result)
        : "m"(a), "m"(b));
    return result;
}

/* ── 5. FRNDINT on already-integer value (idempotent) ────────────────────
 * FLD val + FRNDINT + FRNDINT + FSTP  (4-insn run).
 */
static double frndint_twice(double val) {
    double result;
    __asm__ volatile(
        "fldl   %1\n\t"
        "frndint\n\t"
        "frndint\n\t"
        "fstpl  %0\n"
        : "=m"(result)
        : "m"(val));
    return result;
}

int main(void) {
    uint16_t saved_cw = get_cw();

    /* ── 1. Standalone FRNDINT with each RC mode ─────────────────────────── */
    printf("=== FRNDINT standalone (RC set before run) ===\n");
    {
        set_cw(CW_NEAREST);
        check_f64("RC=nearest  2.5  → 2.0", frndint_standalone(2.5), 2.0);
        check_f64("RC=nearest  3.5  → 4.0", frndint_standalone(3.5), 4.0);
        check_f64("RC=nearest  2.3  → 2.0", frndint_standalone(2.3), 2.0);
        check_f64("RC=nearest -2.5  → -2.0", frndint_standalone(-2.5), -2.0);
        check_f64("RC=nearest -3.5  → -4.0", frndint_standalone(-3.5), -4.0);

        set_cw(CW_FLOOR);
        check_f64("RC=floor    2.9  → 2.0", frndint_standalone(2.9), 2.0);
        check_f64("RC=floor   -2.1  → -3.0", frndint_standalone(-2.1), -3.0);
        check_f64("RC=floor    0.5  → 0.0", frndint_standalone(0.5), 0.0);

        set_cw(CW_CEIL);
        check_f64("RC=ceil     2.1  → 3.0", frndint_standalone(2.1), 3.0);
        check_f64("RC=ceil    -2.9  → -2.0", frndint_standalone(-2.9), -2.0);
        check_f64("RC=ceil    -0.5  → 0.0", frndint_standalone(-0.5), 0.0);

        set_cw(CW_TRUNC);
        check_f64("RC=trunc    2.9  → 2.0", frndint_standalone(2.9), 2.0);
        check_f64("RC=trunc   -2.9  → -2.0", frndint_standalone(-2.9), -2.0);
        check_f64("RC=trunc    0.9  → 0.0", frndint_standalone(0.9), 0.0);
    }

    /* ── 2. FLDCW + FRNDINT in one run ───────────────────────────────────── */
    printf("\n=== FLDCW + FRNDINT in one run ===\n");
    {
        set_cw(CW_NEAREST); /* baseline: nearest */
        check_f64("FLDCW→floor  + FRNDINT  2.9  → 2.0", fldcw_frndint(2.9, CW_FLOOR), 2.0);
        check_f64("FLDCW→ceil   + FRNDINT  2.1  → 3.0", fldcw_frndint(2.1, CW_CEIL), 3.0);
        check_f64("FLDCW→trunc  + FRNDINT -2.9  → -2.0", fldcw_frndint(-2.9, CW_TRUNC), -2.0);
        check_f64("FLDCW→nearest+ FRNDINT  2.5  → 2.0", fldcw_frndint(2.5, CW_NEAREST), 2.0);
    }

    /* ── 3. FRNDINT mid-arithmetic ───────────────────────────────────────── */
    printf("\n=== FRNDINT + arithmetic in one run ===\n");
    {
        set_cw(CW_FLOOR);
        check_f64("floor(2.9) + 0.5 = 2.5", frndint_then_add(2.9, 0.5), 2.5);
        check_f64("floor(-2.1) + 10.0 = 7.0", frndint_then_add(-2.1, 10.0), 7.0);

        set_cw(CW_CEIL);
        check_f64("ceil(2.1) + 0.5 = 3.5", frndint_then_add(2.1, 0.5), 3.5);
    }

    /* ── 4. Double FRNDINT ───────────────────────────────────────────────── */
    printf("\n=== Double FRNDINT in one run ===\n");
    {
        set_cw(CW_FLOOR);
        check_f64("floor(2.9) + floor(3.1) = 5.0", frndint_double(2.9, 3.1), 5.0);
        check_f64("floor(-0.1) + floor(0.1) = -1.0", frndint_double(-0.1, 0.1), -1.0);
    }

    /* ── 5. FRNDINT idempotent (round twice) ─────────────────────────────── */
    printf("\n=== FRNDINT idempotent ===\n");
    {
        set_cw(CW_NEAREST);
        check_f64("round(round(2.7)) = 3.0", frndint_twice(2.7), 3.0);
        check_f64("round(round(4.0)) = 4.0", frndint_twice(4.0), 4.0);

        set_cw(CW_FLOOR);
        check_f64("floor(floor(-2.3)) = -3.0", frndint_twice(-2.3), -3.0);
    }

    /* Restore original control word */
    set_cw(saved_cw);

    printf("\n%s  (%d failure%s)\n", failures == 0 ? "ALL PASS" : "SOME FAILURES", failures,
           failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
