/*
 * test_tag_batch.c -- validate OPT-D2 batched tag word updates
 *
 * Build:  clang -arch x86_64 -O0 -g -o test_tag_batch test_tag_batch.c
 *
 * Tests that deferring pop tag-set-empty updates to run boundaries
 * produces correct results across various push/pop patterns within
 * cache runs (runs of consecutive x87 instructions).
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

static void check_f64(const char* name, double got, double expected) {
    if (as_u64(got) != as_u64(expected)) {
        printf("FAIL  %-60s  got=%.15g  expected=%.15g\n", name, got, expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

static void check_u16(const char* name, uint16_t got, uint16_t expected) {
    if (got != expected) {
        printf("FAIL  %-60s  got=0x%04x  expected=0x%04x\n", name, got, expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

/* ========================================================================= */
/* Section A: Multiple standalone pops via FADDP                             */
/*                                                                           */
/* Push 5 values, pop 4 via faddp, final fstp.  The 4 faddp pops should     */
/* have their tag updates batched to run end.                                */
/* ========================================================================= */

static void test_a_pop_chain_4(void) {
    volatile double r;
    __asm__ volatile(
        "fld1\n\t"
        "fld1\n\t"
        "fld1\n\t"
        "fld1\n\t"
        "fld1\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "fstpl %0\n"
        : "=m"(r));
    check_f64("A: pop_chain_4 (5*fld1 + 4*faddp + fstp = 5.0)", r, 5.0);
}

/* ========================================================================= */
/* Section B: Push-pop cancellation mixed with standalone pops               */
/*                                                                           */
/* fld+faddp pairs cancel push/pop tags; final fstp is a standalone pop.     */
/* ========================================================================= */

static void test_b_cancel_mixed(void) {
    volatile double mem = 2.5;
    volatile double r;
    __asm__ volatile(
        "fld1\n\t"    /* push initial value */
        "fldl %1\n\t" /* push 2.5 (deferred) */
        "faddp\n\t"   /* pop cancels push tag → ST(0) = 3.5 */
        "fldl %1\n\t" /* push 2.5 (deferred) */
        "fmulp\n\t"   /* pop cancels push tag → ST(0) = 8.75 */
        "fstpl %0\n"  /* standalone pop */
        : "=m"(r)
        : "m"(mem));
    check_f64("B: cancel_mixed (1 + 2.5 = 3.5, * 2.5 = 8.75)", r, 8.75);
}

/* ========================================================================= */
/* Section C: Deep stack with many pops                                      */
/* ========================================================================= */

static void test_c_deep_pop(void) {
    volatile double v2 = 2.0, v3 = 3.0, v4 = 4.0, v5 = 5.0, v6 = 6.0;
    volatile double r;
    __asm__ volatile(
        "fldl %1\n\t"
        "fldl %2\n\t"
        "fldl %3\n\t"
        "fldl %4\n\t"
        "fldl %5\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "fstpl %0\n"
        : "=m"(r)
        : "m"(v2), "m"(v3), "m"(v4), "m"(v5), "m"(v6));
    check_f64("C: deep_pop (2+3+4+5+6 = 20.0)", r, 20.0);
}

/* ========================================================================= */
/* Section D: FCOM + FSTSW after deferred pops (flush point test)            */
/*                                                                           */
/* The faddp creates a deferred pop tag. FCOM+FSTSW is a flush point that    */
/* must flush deferred tags before reading status_word.                       */
/* ========================================================================= */

static void test_d_fcom_flush(void) {
    volatile double v2 = 2.0, v3 = 3.0;
    volatile double r;
    uint16_t sw;
    __asm__ volatile(
        "fldl %2\n\t"
        "fldl %3\n\t"
        "faddp\n\t"        /* deferred pop tag → ST(0) = 5.0 */
        "fcom %%st(0)\n\t" /* compare ST(0) with itself → EQ */
        "fnstsw %%ax\n\t"  /* flush point: must flush batched tags */
        "mov %%ax, %1\n\t"
        "fstpl %0\n"
        : "=m"(r), "=r"(sw)
        : "m"(v2), "m"(v3)
        : "ax");
    check_f64("D: fcom_flush value (2+3 = 5.0)", r, 5.0);
    /* EQ: C3=1(bit14), C2=0, C0=0 → bits[14:8] = 0x40xx → (sw >> 8) & 0x45 == 0x40 */
    check_u16("D: fcom_flush status (EQ: C3=1,C2=0,C0=0)", (sw >> 8) & 0x45, 0x40);
}

/* ========================================================================= */
/* Section E: Mixed push-pop-push-pop chains (stress test)                   */
/*                                                                           */
/* 4 alternating fld1+faddp sequences.  Each push-pop pair should cancel.    */
/* ========================================================================= */

static void test_e_cancel_chain(void) {
    volatile double r;
    __asm__ volatile(
        "fld1\n\t" /* base value */
        "fld1\n\t"
        "faddp\n\t" /* cancel → ST(0) = 2 */
        "fld1\n\t"
        "faddp\n\t" /* cancel → ST(0) = 3 */
        "fld1\n\t"
        "faddp\n\t" /* cancel → ST(0) = 4 */
        "fld1\n\t"
        "faddp\n\t" /* cancel → ST(0) = 5 */
        "fstpl %0\n"
        : "=m"(r));
    check_f64("E: cancel_chain (1 + 4*fld1+faddp = 5.0)", r, 5.0);
}

/* ========================================================================= */
/* Section F: FCOMPP (double pop) with batched tags                          */
/* ========================================================================= */

static void test_f_fcompp(void) {
    volatile double v1 = 1.0, v2 = 2.0, v3 = 3.0;
    volatile double r;
    uint16_t sw;
    __asm__ volatile(
        "fldl %2\n\t" /* ST(2) = 1 */
        "fldl %3\n\t" /* ST(1) = 2 */
        "fldl %4\n\t" /* ST(0) = 3 */
        "fcompp\n\t"  /* compare ST(0)=3 vs ST(1)=2, pop both → ST(0) = 1 */
        "fnstsw %%ax\n\t"
        "mov %%ax, %1\n\t"
        "fstpl %0\n"
        : "=m"(r), "=r"(sw)
        : "m"(v1), "m"(v2), "m"(v3)
        : "ax");
    check_f64("F: fcompp value (remaining ST(0) = 1.0)", r, 1.0);
    /* GT: C3=0, C2=0, C0=0 → (sw >> 8) & 0x45 == 0x00 */
    check_u16("F: fcompp status (3 > 2: C3=0,C2=0,C0=0)", (sw >> 8) & 0x45, 0x00);
}

/* ========================================================================= */
/* Section G: Pop chain with subtraction (non-commutative)                   */
/* ========================================================================= */

static void test_g_sub_chain(void) {
    volatile double v10 = 10.0;
    volatile double r;
    __asm__ volatile(
        "fldl %1\n\t" /* ST(0) = 10 */
        "fld1\n\t"    /* ST(0) = 1, ST(1) = 10 */
        "fld1\n\t"    /* ST(0) = 1, ST(1) = 1, ST(2) = 10 */
        "fsubp\n\t"   /* ST(0) = 1-1 = 0, ST(1) = 10 */
        "faddp\n\t"   /* ST(0) = 10+0 = 10 */
        "fstpl %0\n"
        : "=m"(r)
        : "m"(v10));
    check_f64("G: sub_chain (10, push 1, push 1, fsubp, faddp = 10.0)", r, 10.0);
}

/* ========================================================================= */
/* Section H: 6 consecutive pops (maximum batching benefit)                  */
/* ========================================================================= */

static void test_h_deep_pop_6(void) {
    volatile double r;
    __asm__ volatile(
        "fld1\n\t"
        "fld1\n\t"
        "fld1\n\t"
        "fld1\n\t"
        "fld1\n\t"
        "fld1\n\t"
        "fld1\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "fstpl %0\n"
        : "=m"(r));
    check_f64("H: deep_pop_6 (7*fld1 + 6*faddp = 7.0)", r, 7.0);
}

/* ========================================================================= */
/* Section I: 3 net pushes in one IR run                                     */
/*                                                                           */
/* Run 1: FLD+FLD+FLD (top_delta=-3) → emit_x87_tag_set_valid_batch(3)      */
/* MOV break, then Run 2: FSTP+FSTP+FSTP verifies the values are intact.    */
/* ========================================================================= */

static void test_i_net_push_3(void) {
    volatile double a = 2.0, b = 3.0, c = 5.0;
    volatile double r0, r1, r2;
    volatile int dummy = 0;
    __asm__ volatile(
        "fldl %4\n\t"     /* ST(2) = a = 2 */
        "fldl %5\n\t"     /* ST(1) = b = 3 */
        "fldl %6\n\t"     /* ST(0) = c = 5 */
        "movl $0, %3\n\t" /* break IR run */
        "fstpl %0\n\t"    /* r0 = c = 5 */
        "fstpl %1\n\t"    /* r1 = b = 3 */
        "fstpl %2\n\t"    /* r2 = a = 2 */
        : "=m"(r0), "=m"(r1), "=m"(r2), "+m"(dummy)
        : "m"(a), "m"(b), "m"(c));
    check_f64("I: net_push_3 r0 (c = 5.0)", r0, 5.0);
    check_f64("I: net_push_3 r1 (b = 3.0)", r1, 3.0);
    check_f64("I: net_push_3 r2 (a = 2.0)", r2, 2.0);
}

/* ========================================================================= */
/* Section J: 2 net pushes with arithmetic in the same run                   */
/*                                                                           */
/* Run 1: FLD+FLD+FLD+FADDP (3 push, 1 pop → top_delta=-2)                  */
/*   → emit_x87_tag_set_valid_batch(2)                                      */
/* ========================================================================= */

static void test_j_net_push_arith(void) {
    volatile double a = 2.0, b = 3.0, c = 5.0;
    volatile double r0, r1;
    volatile int dummy = 0;
    __asm__ volatile(
        "fldl %3\n\t"     /* ST(0) = a = 2 */
        "fldl %4\n\t"     /* ST(0) = b = 3, ST(1) = 2 */
        "fldl %5\n\t"     /* ST(0) = c = 5, ST(1) = 3, ST(2) = 2 */
        "faddp\n\t"       /* ST(0) = 5+3 = 8, ST(1) = 2 */
        "movl $0, %2\n\t" /* break */
        "fstpl %0\n\t"    /* r0 = 8 */
        "fstpl %1\n\t"    /* r1 = 2 */
        : "=m"(r0), "=m"(r1), "+m"(dummy)
        : "m"(a), "m"(b), "m"(c));
    check_f64("J: net_push_arith r0 (5+3 = 8.0)", r0, 8.0);
    check_f64("J: net_push_arith r1 (a = 2.0)", r1, 2.0);
}

/* ========================================================================= */
/* Section K: 5 net pushes (wider mask, exercises wrap-around path)          */
/*                                                                           */
/* Run 1: 5*FLD1 (top_delta=-5) → emit_x87_tag_set_valid_batch(5)           */
/*   mask = 0x3FF, shifted by top*2 — may wrap past bit 15.                 */
/* ========================================================================= */

static void test_k_net_push_5(void) {
    volatile double r;
    volatile int dummy = 0;
    __asm__ volatile(
        "fld1\n\t"
        "fld1\n\t"
        "fld1\n\t"
        "fld1\n\t"
        "fld1\n\t"        /* 5 pushes, 0 pops */
        "movl $0, %1\n\t" /* break */
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "fstpl %0\n"
        : "=m"(r), "+m"(dummy));
    check_f64("K: net_push_5 (5 * fld1 = 5.0)", r, 5.0);
}

int main(void) {
    test_a_pop_chain_4();
    test_b_cancel_mixed();
    test_c_deep_pop();
    test_d_fcom_flush();
    test_e_cancel_chain();
    test_f_fcompp();
    test_g_sub_chain();
    test_h_deep_pop_6();
    test_i_net_push_3();
    test_j_net_push_arith();
    test_k_net_push_5();

    printf("\n%s: %d failure(s)\n", failures ? "FAILED" : "ALL PASSED", failures);
    return failures ? 1 : 0;
}
