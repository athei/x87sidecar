/*
 * test_fxch_initial.c -- Test FXCH on pre-existing (unresolved) stack values.
 *
 * The bug: FXCH is a compile-time swap of slot_val entries.  If both swapped
 * slots are still initial (negative), the epilogue skips them (val < 0) and
 * the physical x87 register file is never updated.  The fix: resolve() both
 * slots before the swap so they become ReadSt nodes that the epilogue stores.
 *
 * We use non-x87 MOV instructions to split asm blocks into separate IR runs,
 * forcing the second run to operate on pre-existing stack values (same pattern
 * as test_readst_elide.c).
 *
 * IMPORTANT: The IR pipeline requires run_remaining >= 3 (Translator.cpp:119)
 * AND consumed >= 2 (X87IRBuild.cpp:383).  So each Run 2 must contain at
 * least 3 x87 instructions for the IR builder to fire.  For all-initial
 * cases, we use three FXCHs (three transpositions = odd permutation).
 *
 * Build: clang -arch x86_64 -O0 -g -o test_fxch_initial test_fxch_initial.c
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;
static uint64_t as_u64(double d) {
    uint64_t u;
    memcpy(&u, &d, 8);
    return u;
}

static void check(const char* name, double got, double expected) {
    if (as_u64(got) != as_u64(expected)) {
        printf("FAIL  %-55s  got=%.17g  expected=%.17g\n", name, got, expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

/*
 * Test 1: Three FXCHs permuting three unresolved initial slots.
 *
 * Run 1: FLD a, FLD b, FLD c  →  ST(0)=c=5, ST(1)=b=3, ST(2)=a=2
 *   MOV break.
 *
 * Run 2: FXCH ST(2) + FXCH ST(1) + FXCH ST(2)  (consumed=3, all initial)
 *   slot_val starts [-1(d0), -2(d1), -3(d2)]
 *   FXCH ST(2): swap [0]↔[2] → [-3(d2), -2(d1), -1(d0)]
 *   FXCH ST(1): swap [0]↔[1] → [-2(d1), -3(d2), -1(d0)]
 *   FXCH ST(2): swap [0]↔[2] → [-1(d0), -3(d2), -2(d1)]
 *   Net: slot 0 → d0 (unchanged), slot 1 → d2, slot 2 → d1.
 *   Values: ST(0)=c=5 (same), ST(1)=a=2, ST(2)=b=3
 *   MOV break.
 *
 * Run 3: FSTP r0, FSTP r1, FSTP r2
 *
 * Without fix: all negative in epilogue → no stores → physical stack
 * unchanged → Run 3 sees [c=5, b=3, a=2] (original order).
 * Expects: [c=5, a=2, b=3].
 */
static void test_fxch_3_initial(double* r0, double* r1, double* r2) {
    volatile double a = 2.0, b = 3.0, c = 5.0;
    int dummy;
    /* Operands: %0=*r0  %1=*r1  %2=*r2  %3=dummy  |  %4=a  %5=b  %6=c */
    __asm__ volatile(
        /* Run 1: push three values */
        "fldl %4\n\t" /* ST(0)=a=2 */
        "fldl %5\n\t" /* ST(0)=b=3, ST(1)=2 */
        "fldl %6\n\t" /* ST(0)=c=5, ST(1)=3, ST(2)=2 */
        /* Break */
        "movl $0, %3\n\t"
        /* Run 2: three FXCHs (all slots initial, consumed=3) */
        "fxch %%st(2)\n\t" /* [a, b, c] */
        "fxch %%st(1)\n\t" /* [b, a, c] */
        "fxch %%st(2)\n\t" /* [c, a, b]  — net: slots 1,2 swapped */
        /* Break */
        "movl $1, %3\n\t"
        /* Run 3: verify */
        "fstpl %0\n\t" /* r0 = c = 5 */
        "fstpl %1\n\t" /* r1 = a = 2 */
        "fstpl %2\n\t" /* r2 = b = 3 */
        : "=m"(*r0), "=m"(*r1), "=m"(*r2), "+m"(dummy)
        : "m"(a), "m"(b), "m"(c));
}

/*
 * Test 2: Mixed — arithmetic resolves ST(0), then two FXCHs move an
 * unresolved initial slot.
 *
 * Run 2: FADD ST(0),ST(2) + FXCH ST(1) + FXCH ST(2)  (consumed=3)
 *   FADD resolves ST(0)→ReadSt(0) and ST(2)→ReadSt(2), creates FAdd node.
 *   slot_val = [FAdd, -2(d1), ReadSt(2)]
 *
 *   FXCH ST(1): Without fix: swap [0]↔[1] → [-2(d1), FAdd, ReadSt(2)]
 *   FXCH ST(2): Without fix: swap [0]↔[2] → [ReadSt(2), FAdd, -2(d1)]
 *
 *   Epilogue without fix:
 *     d=0: ReadSt(2), init_depth=2, d+top=0. 2≠0 → STORE  (a=2 to slot 0)
 *     d=1: FAdd → STORE  (7 to slot 1)
 *     d=2: val=-2 < 0 → SKIP!  Physical slot 2 still has a=2, should be b=3.
 */
static void test_fxch_mixed(double* r0, double* r1, double* r2) {
    volatile double a = 2.0, b = 3.0, c = 5.0;
    int dummy;
    /* Operands: %0=*r0  %1=*r1  %2=*r2  %3=dummy  |  %4=a  %5=b  %6=c */
    __asm__ volatile(
        /* Run 1 */
        "fldl %4\n\t" /* ST(0)=a=2 */
        "fldl %5\n\t" /* ST(0)=b=3, ST(1)=2 */
        "fldl %6\n\t" /* ST(0)=c=5, ST(1)=3, ST(2)=2 */
        "movl $0, %3\n\t"
        /* Run 2: FADD + 2 FXCHs (consumed=3, one slot stays initial) */
        "fadd %%st(2), %%st\n\t" /* ST(0) = 5+2 = 7 */
        "fxch %%st(1)\n\t"       /* ST(0)=b(init), ST(1)=7 */
        "fxch %%st(2)\n\t"       /* ST(0)=a(ReadSt), ST(1)=7, ST(2)=b(init) */
        "movl $1, %3\n\t"
        /* Run 3: verify */
        "fstpl %0\n\t" /* r0 = a = 2 */
        "fstpl %1\n\t" /* r1 = c+a = 7 */
        "fstpl %2\n\t" /* r2 = b = 3 */
        : "=m"(*r0), "=m"(*r1), "=m"(*r2), "+m"(dummy)
        : "m"(a), "m"(b), "m"(c));
}

/*
 * Test 3: Deep — three FXCHs permuting four unresolved initial slots.
 *
 * Run 2: FXCH ST(3) + FXCH ST(2) + FXCH ST(1)  (consumed=3)
 *   slot_val starts [-1(d0), -2(d1), -3(d2), -4(d3)]
 *   FXCH ST(3): [-4, -2, -3, -1]
 *   FXCH ST(2): [-3, -2, -4, -1]
 *   FXCH ST(1): [-2, -3, -4, -1]
 *   Net: left-rotation: [d0,d1,d2,d3] → [d1,d2,d3,d0]
 *   Values: ST(0)=c, ST(1)=b, ST(2)=a, ST(3)=d
 *
 * Without fix: no stores → Run 3 sees [d,c,b,a]. Expects [c,b,a,d].
 */
static void test_fxch_deep(double* r0, double* r1, double* r2, double* r3) {
    volatile double a = 2.0, b = 3.0, c = 5.0, d = 7.0;
    int dummy;
    /* Operands: %0=*r0 %1=*r1 %2=*r2 %3=*r3 %4=dummy | %5=a %6=b %7=c %8=d */
    __asm__ volatile(
        /* Run 1: push four values */
        "fldl %5\n\t" /* ST(0)=a=2 */
        "fldl %6\n\t" /* ST(0)=b=3 */
        "fldl %7\n\t" /* ST(0)=c=5 */
        "fldl %8\n\t" /* ST(0)=d=7, ST(1)=c, ST(2)=b, ST(3)=a */
        "movl $0, %4\n\t"
        /* Run 2: three FXCHs (consumed=3, all initial, deep targets) */
        "fxch %%st(3)\n\t" /* [a, c, b, d] */
        "fxch %%st(2)\n\t" /* [b, c, a, d] */
        "fxch %%st(1)\n\t" /* [c, b, a, d] */
        "movl $1, %4\n\t"
        /* Run 3: verify */
        "fstpl %0\n\t" /* r0 = c = 5 */
        "fstpl %1\n\t" /* r1 = b = 3 */
        "fstpl %2\n\t" /* r2 = a = 2 */
        "fstpl %3\n\t" /* r3 = d = 7 */
        : "=m"(*r0), "=m"(*r1), "=m"(*r2), "=m"(*r3), "+m"(dummy)
        : "m"(a), "m"(b), "m"(c), "m"(d));
}

int main(void) {
    printf("=== FXCH on pre-existing (initial) stack values ===\n\n");

    {
        double r0, r1, r2;
        test_fxch_3_initial(&r0, &r1, &r2);
        check("3-init: r0 = c = 5 (ST(0) unchanged)", r0, 5.0);
        check("3-init: r1 = a = 2 (moved from ST(2))", r1, 2.0);
        check("3-init: r2 = b = 3 (moved from ST(1))", r2, 3.0);
    }

    printf("\n");

    {
        double r0, r1, r2;
        test_fxch_mixed(&r0, &r1, &r2);
        check("mixed: r0 = a = 2 (ReadSt, swapped to ST(0))", r0, 2.0);
        check("mixed: r1 = c+a = 7 (FAdd, swapped to ST(1))", r1, 7.0);
        check("mixed: r2 = b = 3 (initial, swapped to ST(2))", r2, 3.0);
    }

    printf("\n");

    {
        double r0, r1, r2, r3;
        test_fxch_deep(&r0, &r1, &r2, &r3);
        check("deep: r0 = c = 5 (rotated from ST(1))", r0, 5.0);
        check("deep: r1 = b = 3 (rotated from ST(2))", r1, 3.0);
        check("deep: r2 = a = 2 (rotated from ST(3))", r2, 2.0);
        check("deep: r3 = d = 7 (rotated from ST(0))", r3, 7.0);
    }

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
