/*
 * test_fcomi.c -- validate FCOMI / FCOMIP / FUCOMI / FUCOMIP via IR pipeline
 *
 * Build:  clang -arch x86_64 -O0 -g -o test_fcomi test_fcomi.c
 *
 * FCOMI sets NZCV (ZF/PF/CF) directly in EFLAGS rather than writing
 * the x87 status word:
 *
 *   ST(0) >  ST(i)  →  ZF=0 CF=0 PF=0
 *   ST(0) <  ST(i)  →  ZF=0 CF=1 PF=0
 *   ST(0) == ST(i)  →  ZF=1 CF=0 PF=0
 *   Unordered       →  ZF=1 CF=1 PF=1
 *
 * We test using SETA/SETB/SETE/SETP after FCOMI to read CF/ZF/PF.
 *
 * FCOMIP is the popping variant (pops ST(0) after the compare).
 * We verify the pop by checking that the original ST(1) value becomes
 * the new ST(0) after FCOMIP.
 */

#include <stdint.h>
#include <stdio.h>

static int failures = 0;

static void check(const char* name, int got, int expected) {
    if (got != expected) {
        printf("FAIL  %-70s  got=%d  expected=%d\n", name, got, expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

/* ── Section A: FCOMI (non-popping) ─────────────────────────────────────── */

/* GT: ST(0)=3.0 > ST(1)=1.0 → ZF=0 CF=0  → SETA=1, SETB=0, SETE=0 */
static void test_fcomi_gt(void) {
    double a = 3.0, b = 1.0;
    uint8_t above = 0, below = 0, equal = 0;
    __asm__ volatile(
        "fldl  %3\n\t"      /* push b=1.0; ST(0)=1.0         */
        "fldl  %2\n\t"      /* push a=3.0; ST(0)=3.0 ST(1)=1.0 */
        "fcomi %%st(1)\n\t" /* compare ST(0) vs ST(1)        */
        "seta  %0\n\t"      /* CF=0 and ZF=0 → above         */
        "setb  %1\n\t"      /* CF=1 → below                  */
        "sete  %4\n\t"      /* ZF=1 → equal                  */
        "fstp  %%st(0)\n\t"
        "fstp  %%st(0)\n"
        : "=r"(above), "=r"(below), "=m"(a), "=m"(b), "=r"(equal)
        : "m"(a), "m"(b)
        : "cc", "st");
    check("A/fcomi GT (3.0 > 1.0) seta=1", above, 1);
    check("A/fcomi GT (3.0 > 1.0) setb=0", below, 0);
    check("A/fcomi GT (3.0 > 1.0) sete=0", equal, 0);
}

/* LT: ST(0)=1.0 < ST(1)=3.0 → ZF=0 CF=1  → SETA=0, SETB=1, SETE=0 */
static void test_fcomi_lt(void) {
    double a = 1.0, b = 3.0;
    uint8_t above = 0, below = 0, equal = 0;
    __asm__ volatile(
        "fldl  %3\n\t"
        "fldl  %2\n\t"
        "fcomi %%st(1)\n\t"
        "seta  %0\n\t"
        "setb  %1\n\t"
        "sete  %4\n\t"
        "fstp  %%st(0)\n\t"
        "fstp  %%st(0)\n"
        : "=r"(above), "=r"(below), "=m"(a), "=m"(b), "=r"(equal)
        : "m"(a), "m"(b)
        : "cc", "st");
    check("A/fcomi LT (1.0 < 3.0) seta=0", above, 0);
    check("A/fcomi LT (1.0 < 3.0) setb=1", below, 1);
    check("A/fcomi LT (1.0 < 3.0) sete=0", equal, 0);
}

/* EQ: ST(0)=2.0 == ST(1)=2.0 → ZF=1 CF=0 → SETA=0, SETB=0, SETE=1 */
static void test_fcomi_eq(void) {
    double a = 2.0, b = 2.0;
    uint8_t above = 0, below = 0, equal = 0;
    __asm__ volatile(
        "fldl  %3\n\t"
        "fldl  %2\n\t"
        "fcomi %%st(1)\n\t"
        "seta  %0\n\t"
        "setb  %1\n\t"
        "sete  %4\n\t"
        "fstp  %%st(0)\n\t"
        "fstp  %%st(0)\n"
        : "=r"(above), "=r"(below), "=m"(a), "=m"(b), "=r"(equal)
        : "m"(a), "m"(b)
        : "cc", "st");
    check("A/fcomi EQ (2.0 == 2.0) seta=0", above, 0);
    check("A/fcomi EQ (2.0 == 2.0) setb=0", below, 0);
    check("A/fcomi EQ (2.0 == 2.0) sete=1", equal, 1);
}

/* Unordered: ST(0)=NaN vs ST(1)=1.0 → ZF=1 CF=1 PF=1 */
static void test_fcomi_unordered(void) {
    double nan_val = __builtin_nan(""), b = 1.0;
    uint8_t parity = 0, below = 0, equal = 0;
    __asm__ volatile(
        "fldl  %3\n\t"
        "fldl  %2\n\t"
        "fcomi %%st(1)\n\t"
        "setp  %0\n\t" /* PF=1 → unordered */
        "setb  %1\n\t"
        "sete  %4\n\t"
        "fstp  %%st(0)\n\t"
        "fstp  %%st(0)\n"
        : "=r"(parity), "=r"(below), "=m"(nan_val), "=m"(b), "=r"(equal)
        : "m"(nan_val), "m"(b)
        : "cc", "st");
    check("A/fcomi UN (NaN vs 1.0) setp=1", parity, 1);
    check("A/fcomi UN (NaN vs 1.0) setb=1", below, 1);
    check("A/fcomi UN (NaN vs 1.0) sete=1", equal, 1);
}

/* ── Section B: FCOMIP (popping) ────────────────────────────────────────── */

/*
 * FCOMIP compares ST(0) vs ST(i) then pops ST(0).
 * After the pop, the original ST(1) becomes the new ST(0).
 *
 * Test: push 9.0 then 5.0, so ST(0)=5.0 ST(1)=9.0.
 *   fcomip ST(1): 5.0 < 9.0 → CF=1 (setb=1), then pop ST(0)=5.0.
 *   Now ST(0) should be 9.0. We read it with fstp into a double variable.
 */
static void test_fcomip_pops(void) {
    double a = 5.0, b = 9.0;
    uint8_t below = 0;
    double st0_after = 0.0;
    __asm__ volatile(
        "fldl  %2\n\t"       /* push b=9.0; ST(0)=9.0           */
        "fldl  %1\n\t"       /* push a=5.0; ST(0)=5.0 ST(1)=9.0 */
        "fcomip %%st(1)\n\t" /* compare, pop → ST(0)=9.0        */
        "setb  %3\n\t"       /* CF=1 (5.0 < 9.0)                */
        "fstpl %0\n"         /* read new ST(0) = 9.0            */
        : "=m"(st0_after), "=m"(a), "=m"(b), "=r"(below)
        : "m"(a), "m"(b)
        : "cc", "st");
    check("B/fcomip (5.0 < 9.0) setb=1", below, 1);
    check("B/fcomip pops ST(0): new ST(0)=9.0", (int)(st0_after == 9.0), 1);
}

/* ── Section C: FUCOMI / FUCOMIP (unordered-quiet variants) ─────────────── */

/* FUCOMI GT: same flag semantics as FCOMI for non-NaN */
static void test_fucomi_gt(void) {
    double a = 4.0, b = 2.0;
    uint8_t above = 0;
    __asm__ volatile(
        "fldl  %2\n\t"
        "fldl  %1\n\t"
        "fucomi %%st(1)\n\t"
        "seta  %0\n\t"
        "fstp  %%st(0)\n\t"
        "fstp  %%st(0)\n"
        : "=r"(above), "=m"(a), "=m"(b)
        : "m"(a), "m"(b)
        : "cc", "st");
    check("C/fucomi GT (4.0 > 2.0) seta=1", above, 1);
}

/* FUCOMIP: unordered+popping, GT case */
static void test_fucomip_pops(void) {
    double a = 7.0, b = 3.0;
    uint8_t above = 0;
    double st0_after = 0.0;
    __asm__ volatile(
        "fldl  %2\n\t"        /* push b=3.0 */
        "fldl  %1\n\t"        /* push a=7.0; ST(0)=7.0 ST(1)=3.0 */
        "fucomip %%st(1)\n\t" /* compare, pop → ST(0)=3.0 */
        "seta  %3\n\t"        /* 7.0 > 3.0 → CF=0 ZF=0 → above */
        "fstpl %0\n"
        : "=m"(st0_after), "=m"(a), "=m"(b), "=r"(above)
        : "m"(a), "m"(b)
        : "cc", "st");
    check("C/fucomip GT (7.0 > 3.0) seta=1", above, 1);
    check("C/fucomip pops ST(0): new ST(0)=3.0", (int)(st0_after == 3.0), 1);
}

/* ── main ───────────────────────────────────────────────────────────────── */

int main(void) {
    test_fcomi_gt();
    test_fcomi_lt();
    test_fcomi_eq();
    test_fcomi_unordered();
    test_fcomip_pops();
    test_fucomi_gt();
    test_fucomip_pops();

    if (failures == 0)
        printf("\nAll tests passed.\n");
    else
        printf("\n%d test(s) FAILED.\n", failures);

    return failures ? 1 : 0;
}
