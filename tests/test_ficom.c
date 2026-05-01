/*
 * test_ficom.c — exercise FICOM / FICOMP (integer compare) variants.
 *
 * Each test function performs one x87 integer compare instruction and returns
 * the masked x87 status-word CC bits as a uint16_t:
 *
 *   result & 0x4500
 *         bit 14 = C3   (equal)
 *         bit 10 = C2   (unordered / parity)
 *         bit  8 = C0   (below / carry)
 *
 * Expected values:
 *   ST(0) > src   0x0000    (C3=0, C2=0, C0=0)
 *   ST(0) < src   0x0100    (C3=0, C2=0, C0=1)
 *   ST(0) = src   0x4000    (C3=1, C2=0, C0=0)
 *   Unordered     0x4500    (C3=1, C2=1, C0=1)
 *
 * Compile (x86 / i386 target, e.g. under Rosetta):
 *   gcc -O0 -m32 -o test_ficom test_ficom.c && ./test_ficom
 */

#include <stdint.h>
#include <stdio.h>

/* Read x87 status-word CC bits after a compare. */
#define READ_SW(var)           \
    uint16_t var;              \
    __asm__ volatile(          \
        "fnstsw %%ax\n"        \
        "andw $0x4500, %%ax\n" \
        "movw %%ax, %0\n"      \
        : "=m"(var)            \
        :                      \
        : "ax")

/* ===========================================================================
 * FICOM m16int  —  DE /2  —  compare ST(0) with int16 memory, no pop
 * =========================================================================== */

/* 3.0 > 1 — expected 0x0000 */
static uint16_t test_ficom_m16_gt(void) {
    double st0 = 3.0;
    int16_t src = 1;
    __asm__ volatile(
        "fldl  %0\n"
        "ficoms %1\n"
        :
        : "m"(st0), "m"(src));
    READ_SW(cc);
    __asm__ volatile("fstp %%st(0)\n" : : : "st");
    return cc;
}

/* 1.0 < 3 — expected 0x0100 */
static uint16_t test_ficom_m16_lt(void) {
    double st0 = 1.0;
    int16_t src = 3;
    __asm__ volatile(
        "fldl  %0\n"
        "ficoms %1\n"
        :
        : "m"(st0), "m"(src));
    READ_SW(cc);
    __asm__ volatile("fstp %%st(0)\n" : : : "st");
    return cc;
}

/* 2.0 = 2 — expected 0x4000 */
static uint16_t test_ficom_m16_eq(void) {
    double st0 = 2.0;
    int16_t src = 2;
    __asm__ volatile(
        "fldl  %0\n"
        "ficoms %1\n"
        :
        : "m"(st0), "m"(src));
    READ_SW(cc);
    __asm__ volatile("fstp %%st(0)\n" : : : "st");
    return cc;
}

/* NaN vs 1 — expected 0x4500 */
static uint16_t test_ficom_m16_un(void) {
    double nan_val = __builtin_nan("");
    int16_t src = 1;
    __asm__ volatile(
        "fldl  %0\n"
        "ficoms %1\n"
        :
        : "m"(nan_val), "m"(src));
    READ_SW(cc);
    __asm__ volatile("fstp %%st(0)\n" : : : "st");
    return cc;
}

/* ===========================================================================
 * FICOM m32int  —  DA /2  —  compare ST(0) with int32 memory, no pop
 * =========================================================================== */

/* 3.0 > 1 — expected 0x0000 */
static uint16_t test_ficom_m32_gt(void) {
    double st0 = 3.0;
    int32_t src = 1;
    __asm__ volatile(
        "fldl  %0\n"
        "ficoml %1\n"
        :
        : "m"(st0), "m"(src));
    READ_SW(cc);
    __asm__ volatile("fstp %%st(0)\n" : : : "st");
    return cc;
}

/* 1.0 < 3 — expected 0x0100 */
static uint16_t test_ficom_m32_lt(void) {
    double st0 = 1.0;
    int32_t src = 3;
    __asm__ volatile(
        "fldl  %0\n"
        "ficoml %1\n"
        :
        : "m"(st0), "m"(src));
    READ_SW(cc);
    __asm__ volatile("fstp %%st(0)\n" : : : "st");
    return cc;
}

/* 2.0 = 2 — expected 0x4000 */
static uint16_t test_ficom_m32_eq(void) {
    double st0 = 2.0;
    int32_t src = 2;
    __asm__ volatile(
        "fldl  %0\n"
        "ficoml %1\n"
        :
        : "m"(st0), "m"(src));
    READ_SW(cc);
    __asm__ volatile("fstp %%st(0)\n" : : : "st");
    return cc;
}

/* NaN vs 1 — expected 0x4500 */
static uint16_t test_ficom_m32_un(void) {
    double nan_val = __builtin_nan("");
    int32_t src = 1;
    __asm__ volatile(
        "fldl  %0\n"
        "ficoml %1\n"
        :
        : "m"(nan_val), "m"(src));
    READ_SW(cc);
    __asm__ volatile("fstp %%st(0)\n" : : : "st");
    return cc;
}

/* ===========================================================================
 * FICOMP m16int  —  DE /3  —  compare ST(0) with int16 memory, pop once
 * After the pop, the stack is empty — no teardown needed.
 * =========================================================================== */

/* 3.0 > 1 — expected 0x0000 */
static uint16_t test_ficomp_m16_gt(void) {
    double st0 = 3.0;
    int16_t src = 1;
    __asm__ volatile(
        "fldl   %0\n"
        "ficomps %1\n"
        :
        : "m"(st0), "m"(src));
    READ_SW(cc);
    return cc;
}

/* 1.0 < 3 — expected 0x0100 */
static uint16_t test_ficomp_m16_lt(void) {
    double st0 = 1.0;
    int16_t src = 3;
    __asm__ volatile(
        "fldl   %0\n"
        "ficomps %1\n"
        :
        : "m"(st0), "m"(src));
    READ_SW(cc);
    return cc;
}

/* 2.0 = 2 — expected 0x4000 */
static uint16_t test_ficomp_m16_eq(void) {
    double st0 = 2.0;
    int16_t src = 2;
    __asm__ volatile(
        "fldl   %0\n"
        "ficomps %1\n"
        :
        : "m"(st0), "m"(src));
    READ_SW(cc);
    return cc;
}

/* NaN vs 1 — expected 0x4500 */
static uint16_t test_ficomp_m16_un(void) {
    double nan_val = __builtin_nan("");
    int16_t src = 1;
    __asm__ volatile(
        "fldl   %0\n"
        "ficomps %1\n"
        :
        : "m"(nan_val), "m"(src));
    READ_SW(cc);
    return cc;
}

/* ===========================================================================
 * FICOMP m32int  —  DA /3  —  compare ST(0) with int32 memory, pop once
 * =========================================================================== */

/* 3.0 > 1 — expected 0x0000 */
static uint16_t test_ficomp_m32_gt(void) {
    double st0 = 3.0;
    int32_t src = 1;
    __asm__ volatile(
        "fldl   %0\n"
        "ficompl %1\n"
        :
        : "m"(st0), "m"(src));
    READ_SW(cc);
    return cc;
}

/* 1.0 < 3 — expected 0x0100 */
static uint16_t test_ficomp_m32_lt(void) {
    double st0 = 1.0;
    int32_t src = 3;
    __asm__ volatile(
        "fldl   %0\n"
        "ficompl %1\n"
        :
        : "m"(st0), "m"(src));
    READ_SW(cc);
    return cc;
}

/* 2.0 = 2 — expected 0x4000 */
static uint16_t test_ficomp_m32_eq(void) {
    double st0 = 2.0;
    int32_t src = 2;
    __asm__ volatile(
        "fldl   %0\n"
        "ficompl %1\n"
        :
        : "m"(st0), "m"(src));
    READ_SW(cc);
    return cc;
}

/* NaN vs 1 — expected 0x4500 */
static uint16_t test_ficomp_m32_un(void) {
    double nan_val = __builtin_nan("");
    int32_t src = 1;
    __asm__ volatile(
        "fldl   %0\n"
        "ficompl %1\n"
        :
        : "m"(nan_val), "m"(src));
    READ_SW(cc);
    return cc;
}

/* ===========================================================================
 * Test table and harness
 * =========================================================================== */
typedef struct {
    const char* name;
    uint16_t (*fn)(void);
    uint16_t expected;
} TestCase;

int main(void) {
    TestCase tests[] = {
        /* FICOM m16int */
        {"ficom  m16int  DE /2  GT  3.0>1", test_ficom_m16_gt, 0x0000},
        {"ficom  m16int  DE /2  LT  1.0<3", test_ficom_m16_lt, 0x0100},
        {"ficom  m16int  DE /2  EQ  2.0=2", test_ficom_m16_eq, 0x4000},
        {"ficom  m16int  DE /2  UN  NaN,1", test_ficom_m16_un, 0x4500},

        /* FICOM m32int */
        {"ficom  m32int  DA /2  GT  3.0>1", test_ficom_m32_gt, 0x0000},
        {"ficom  m32int  DA /2  LT  1.0<3", test_ficom_m32_lt, 0x0100},
        {"ficom  m32int  DA /2  EQ  2.0=2", test_ficom_m32_eq, 0x4000},
        {"ficom  m32int  DA /2  UN  NaN,1", test_ficom_m32_un, 0x4500},

        /* FICOMP m16int */
        {"ficomp m16int  DE /3  GT  3.0>1", test_ficomp_m16_gt, 0x0000},
        {"ficomp m16int  DE /3  LT  1.0<3", test_ficomp_m16_lt, 0x0100},
        {"ficomp m16int  DE /3  EQ  2.0=2", test_ficomp_m16_eq, 0x4000},
        {"ficomp m16int  DE /3  UN  NaN,1", test_ficomp_m16_un, 0x4500},

        /* FICOMP m32int */
        {"ficomp m32int  DA /3  GT  3.0>1", test_ficomp_m32_gt, 0x0000},
        {"ficomp m32int  DA /3  LT  1.0<3", test_ficomp_m32_lt, 0x0100},
        {"ficomp m32int  DA /3  EQ  2.0=2", test_ficomp_m32_eq, 0x4000},
        {"ficomp m32int  DA /3  UN  NaN,1", test_ficomp_m32_un, 0x4500},
    };

    int pass = 0, fail = 0;
    int n = (int)(sizeof(tests) / sizeof(tests[0]));
    for (int i = 0; i < n; i++) {
        uint16_t got = tests[i].fn();
        int ok = (got == tests[i].expected);
        printf("%s  got=0x%04x  expected=0x%04x  %s\n", tests[i].name, (unsigned)got,
               (unsigned)tests[i].expected, ok ? "PASS" : "FAIL");
        ok ? pass++ : fail++;
    }

    printf("\n%d/%d passed\n", pass, n);
    return fail ? 1 : 0;
}
