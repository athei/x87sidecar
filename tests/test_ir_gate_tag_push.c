/*
 * test_ir_gate_tag_push.c -- regression tests for the IR-gate tag_push_pending
 * flush bug.
 *
 * Build:  clang -arch x86_64 -O0 -g -o test_ir_gate_tag_push test_ir_gate_tag_push.c
 *
 * The IR-gate's tag_push branch (Translator.cpp:197-232) flushes a deferred
 * tag-valid update at the entry-time TOP slot, then falls through to
 * compile_run.  When the IR run that follows has top_delta != 0, the slot the
 * gate just wrote no longer corresponds to where the IR's epilogue emits its
 * own tag updates.  This test exercises that interaction across several
 * shapes and probes both the numeric stack values (FSTP) and the tag word
 * (FNSTENV) so a corruption is observable.
 *
 * Run with X87_GATE_FLUSH_THRESHOLD_TAG_PUSH=3 to maximise the chance the
 * gate's tag_push branch fires; default threshold=8 makes the bug very rare.
 * Phase 5 (X87_DISABLE_HOOK=1) is the bit-exact reference and must always
 * pass.
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
        printf("FAIL  %-60s  got=%.17g  expected=%.17g\n", name, got, expected);
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
/* A — Matrix-vector dot product (single row).  WoW character-rotation hot   */
/* pattern.  Long single run gives the gate's tag_push branch many chances   */
/* to fire at threshold=3.                                                   */
/* ========================================================================= */

static void test_a_mv_dot(void) {
    volatile double m00 = 1.5, m01 = -0.5, m02 = 0.25;
    volatile double v0 = 2.0, v1 = 3.0, v2 = 4.0;
    volatile double r;
    __asm__ volatile(
        "fldl %[m00]\n\t"
        "fmull %[v0]\n\t"
        "fldl %[m01]\n\t"
        "fmull %[v1]\n\t"
        "faddp\n\t"
        "fldl %[m02]\n\t"
        "fmull %[v2]\n\t"
        "faddp\n\t"
        "fstpl %[r]\n"
        : [r] "=m"(r)
        : [m00] "m"(m00), [m01] "m"(m01), [m02] "m"(m02), [v0] "m"(v0), [v1] "m"(v1), [v2] "m"(v2));
    check_f64("A: mv_dot (1.5*2 + -0.5*3 + 0.25*4 = 2.5)", r, 2.5);
}

/* ========================================================================= */
/* B — 3x3 matrix * vector, all three rows in one IR-eligible run.  Three   */
/* outputs surface stale tag bits if the wrong slot was marked valid.       */
/* ========================================================================= */

static void test_b_mat3x3_mv(void) {
    /* Diagonal-2 matrix: r = 2 * v. */
    volatile double m[9] = {2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0};
    volatile double v[3] = {1.0, 2.0, 3.0};
    volatile double r0 = 0, r1 = 0, r2 = 0;
    volatile int dummy = 0;
    __asm__ volatile(
        /* row0 . v */
        "fldl %[m0]\n\t"
        "fmull %[v0]\n\t"
        "fldl %[m1]\n\t"
        "fmull %[v1]\n\t"
        "faddp\n\t"
        "fldl %[m2]\n\t"
        "fmull %[v2]\n\t"
        "faddp\n\t"
        /* row1 . v */
        "fldl %[m3]\n\t"
        "fmull %[v0]\n\t"
        "fldl %[m4]\n\t"
        "fmull %[v1]\n\t"
        "faddp\n\t"
        "fldl %[m5]\n\t"
        "fmull %[v2]\n\t"
        "faddp\n\t"
        /* row2 . v */
        "fldl %[m6]\n\t"
        "fmull %[v0]\n\t"
        "fldl %[m7]\n\t"
        "fmull %[v1]\n\t"
        "faddp\n\t"
        "fldl %[m8]\n\t"
        "fmull %[v2]\n\t"
        "faddp\n\t"
        "movl $0, %[dummy]\n\t" /* break IR run before stores */
        "fstpl %[r2]\n\t"       /* r2 = row2 . v */
        "fstpl %[r1]\n\t"       /* r1 = row1 . v */
        "fstpl %[r0]\n"         /* r0 = row0 . v */
        : [r0] "=m"(r0), [r1] "=m"(r1), [r2] "=m"(r2), [dummy] "+m"(dummy)
        : [m0] "m"(m[0]), [m1] "m"(m[1]), [m2] "m"(m[2]), [m3] "m"(m[3]), [m4] "m"(m[4]),
          [m5] "m"(m[5]), [m6] "m"(m[6]), [m7] "m"(m[7]), [m8] "m"(m[8]), [v0] "m"(v[0]),
          [v1] "m"(v[1]), [v2] "m"(v[2]));
    check_f64("B: mat3x3 r0 (2*1 = 2.0)", r0, 2.0);
    check_f64("B: mat3x3 r1 (2*2 = 4.0)", r1, 4.0);
    check_f64("B: mat3x3 r2 (2*3 = 6.0)", r2, 6.0);
}

/* ========================================================================= */
/* C — top_delta == +1 (net pop): 4 pushes followed by 4 pops via faddp +    */
/* fstpl.  Net pop relative to entry is 1 (the fstpl).  Probes whether the  */
/* IR's tag_set_empty batch overwrites the slot the gate marked valid.      */
/* ========================================================================= */

static void test_c_net_pop_1(void) {
    volatile double a = 10.0, b = 20.0, c = 30.0, d = 40.0;
    volatile double r;
    __asm__ volatile(
        "fldl %[a]\n\t" /* push a */
        "fldl %[b]\n\t" /* push b */
        "fldl %[c]\n\t" /* push c */
        "fldl %[d]\n\t" /* push d (4 pushes) */
        "faddp\n\t"     /* ST(0) = c+d, ST(1) = b, ST(2) = a */
        "faddp\n\t"     /* ST(0) = b+c+d, ST(1) = a */
        "faddp\n\t"     /* ST(0) = a+b+c+d (3 pops) */
        "fstpl %[r]\n"  /* pop and store */
        : [r] "=m"(r)
        : [a] "m"(a), [b] "m"(b), [c] "m"(c), [d] "m"(d));
    check_f64("C: net_pop_1 (10+20+30+40 = 100)", r, 100.0);
}

/* ========================================================================= */
/* D — top_delta == -1 (net push) with long arith run.                       */
/* ========================================================================= */

static void test_d_net_push_1(void) {
    volatile double a = 1.0, b = 2.0, c = 3.0, d = 4.0, e = 5.0;
    volatile double r0;
    volatile int dummy = 0;
    __asm__ volatile(
        "fldl %[a]\n\t"
        "fldl %[b]\n\t"
        "faddp\n\t" /* ST(0) = a+b = 3 */
        "fldl %[c]\n\t"
        "fldl %[d]\n\t"
        "faddp\n\t"     /* ST(0) = c+d = 7, ST(1) = 3 */
        "fldl %[e]\n\t" /* push e — net push = +1 vs entry */
        "movl $0, %[dummy]\n\t"
        "fstpl %[r0]\n\t"  /* r0 = e = 5 */
        "fstp %%st(0)\n\t" /* drop ST(1) = 7 */
        "fstp %%st(0)\n"   /* drop ST(2) = 3 */
        : [r0] "=m"(r0), [dummy] "+m"(dummy)
        : [a] "m"(a), [b] "m"(b), [c] "m"(c), [d] "m"(d), [e] "m"(e)
        : "st");
    check_f64("D: net_push_1 r0 (e = 5.0)", r0, 5.0);
}

/* ========================================================================= */
/* E — Probe tag word via FNSTENV after pushing 3 values.  3 valid + 5      */
/* empty pattern is invariant regardless of TOP rotation.                   */
/* ========================================================================= */

static void test_e_tag_word_after_run(void) {
    volatile double a = 1.5, b = 2.5, c = 3.5;
    volatile uint8_t env[28];
    __asm__ volatile(
        "fldl %[a]\n\t"
        "fldl %[b]\n\t"
        "fldl %[c]\n\t"
        "fnstenv %[env]\n\t"
        "fstp %%st(0)\n\t"
        "fstp %%st(0)\n\t"
        "fstp %%st(0)\n\t"
        : [env] "=m"(env)
        : [a] "m"(a), [b] "m"(b), [c] "m"(c)
        : "st");
    uint16_t tagw;
    memcpy(&tagw, (const void*)&env[8], 2);
    int valid_count = 0;
    int empty_count = 0;
    for (int i = 0; i < 8; i++) {
        const uint16_t bits = (tagw >> (2 * i)) & 0x3;
        if (bits == 0) {
            valid_count++;
        } else if (bits == 3) {
            empty_count++;
        }
    }
    if (valid_count == 3 && empty_count == 5) {
        printf("PASS  E: tag_word_after_run (3 valid, 5 empty)\n");
    } else {
        printf("FAIL  E: tag_word_after_run  tagw=0x%04x valid=%d empty=%d\n", tagw, valid_count,
               empty_count);
        failures++;
    }
}

/* ========================================================================= */
/* F — Tag word after a long balanced run (top_delta == 0).  Internal       */
/* push+pop pairs around a base value.  Net 1 valid slot expected.         */
/* ========================================================================= */

static void test_f_tag_word_balanced_run(void) {
    volatile double a = 7.0, b = 11.0;
    volatile uint8_t env[28];
    __asm__ volatile(
        "fldl %[a]\n\t" /* push a */
        "fldl %[b]\n\t" /* push b */
        "faddp\n\t"     /* pop, ST(0) = a + b */
        "fldl %[b]\n\t" /* push b */
        "faddp\n\t"     /* pop, ST(0) = a + 2b */
        "fldl %[b]\n\t" /* push b */
        "faddp\n\t"     /* pop, ST(0) = a + 3b */
        "fnstenv %[env]\n\t"
        "fstp %%st(0)\n\t"
        : [env] "=m"(env)
        : [a] "m"(a), [b] "m"(b)
        : "st");
    uint16_t tagw;
    memcpy(&tagw, (const void*)&env[8], 2);
    int valid_count = 0;
    int empty_count = 0;
    for (int i = 0; i < 8; i++) {
        const uint16_t bits = (tagw >> (2 * i)) & 0x3;
        if (bits == 0) {
            valid_count++;
        } else if (bits == 3) {
            empty_count++;
        }
    }
    if (valid_count == 1 && empty_count == 7) {
        printf("PASS  F: tag_word_balanced_run (1 valid, 7 empty)\n");
    } else {
        printf("FAIL  F: tag_word_balanced_run  tagw=0x%04x valid=%d empty=%d\n", tagw, valid_count,
               empty_count);
        failures++;
    }
}

/* ========================================================================= */
/* G — FXAM observer probe.  After a long run that leaves 3 values on stack, */
/* probe each ST(i) tag classification via FXAM.                             */
/* ========================================================================= */

static void test_g_fxam_after_gate(void) {
    volatile double a = 3.14, b = 2.71;
    volatile uint16_t sw0, sw1, sw2;
    volatile uint8_t env_exit[28];
    __asm__ volatile(
        "fldl %[a]\n\t" /* push a */
        "fldl %[b]\n\t" /* push b */
        "fmulp\n\t"     /* pop, ST(0) = a*b */
        "fldl %[a]\n\t" /* push a */
        "fldl %[b]\n\t" /* push b */
        "faddp\n\t"     /* pop, ST(0) = a+b, ST(1) = a*b */
        "fldl %[a]\n\t" /* push a — ST(0)=a, ST(1)=a+b, ST(2)=a*b */
        "fxam\n\t"
        "fnstsw %%ax\n\t"
        "movw %%ax, %[sw0]\n\t"
        "fstp %%st(0)\n\t"
        "fxam\n\t"
        "fnstsw %%ax\n\t"
        "movw %%ax, %[sw1]\n\t"
        "fstp %%st(0)\n\t"
        "fxam\n\t"
        "fnstsw %%ax\n\t"
        "movw %%ax, %[sw2]\n\t"
        "fstp %%st(0)\n\t"
        "fnstenv %[env_exit]\n\t" /* probe tag word at function exit */
        : [sw0] "=m"(sw0), [sw1] "=m"(sw1), [sw2] "=m"(sw2), [env_exit] "=m"(env_exit)
        : [a] "m"(a), [b] "m"(b)
        : "ax", "st");
    uint16_t tagw_exit;
    memcpy(&tagw_exit, (const void*)&env_exit[8], 2);
    check_u16("G: tag_word at exit (all empty)", tagw_exit, 0xffff);
    /* All three should be Normal (C3=0, C2=1, C0=0).  Mask 0x4500 ignores C1.*/
    check_u16("G: fxam ST(0) (Normal)", sw0 & 0x4500, 0x0400);
    check_u16("G: fxam ST(1) (Normal)", sw1 & 0x4500, 0x0400);
    check_u16("G: fxam ST(2) (Normal)", sw2 & 0x4500, 0x0400);
}

/* ========================================================================= */
/* H — Long accumulator chain (16 ops): max chances for the gate's tag_push */
/* branch to fire at threshold=3.                                            */
/* ========================================================================= */

static void test_h_long_acc_chain(void) {
    volatile double v = 0.5;
    volatile double r;
    __asm__ volatile(
        "fldl %[v]\n\t"
        "fldl %[v]\n\t"
        "faddp\n\t"
        "fldl %[v]\n\t"
        "faddp\n\t"
        "fldl %[v]\n\t"
        "faddp\n\t"
        "fldl %[v]\n\t"
        "faddp\n\t"
        "fldl %[v]\n\t"
        "faddp\n\t"
        "fldl %[v]\n\t"
        "faddp\n\t"
        "fldl %[v]\n\t"
        "faddp\n\t"
        "fstpl %[r]\n"
        : [r] "=m"(r)
        : [v] "m"(v));
    check_f64("H: long_acc_chain (8 * 0.5 = 4.0)", r, 4.0);
}

/* ========================================================================= */
/* I — 32-bit float chain (WoW most-common shape).                           */
/* ========================================================================= */

static void test_i_mixed_32bit_chain(void) {
    volatile float fa = 0.5f, fb = 0.25f, fc = 0.125f;
    volatile float va = 8.0f, vb = 16.0f, vc = 32.0f;
    volatile float r;
    __asm__ volatile(
        "flds %[fa]\n\t"
        "fmuls %[va]\n\t"
        "flds %[fb]\n\t"
        "fmuls %[vb]\n\t"
        "faddp\n\t"
        "flds %[fc]\n\t"
        "fmuls %[vc]\n\t"
        "faddp\n\t"
        "fstps %[r]\n"
        : [r] "=m"(r)
        : [fa] "m"(fa), [fb] "m"(fb), [fc] "m"(fc), [va] "m"(va), [vb] "m"(vb), [vc] "m"(vc));
    /* 0.5*8 + 0.25*16 + 0.125*32 = 4 + 4 + 4 = 12 */
    if (r != 12.0f) {
        printf("FAIL  I: mixed_32bit_chain  got=%.9g expected=12.0\n", (double)r);
        failures++;
    } else {
        printf("PASS  I: mixed_32bit_chain (0.5*8 + 0.25*16 + 0.125*32 = 12.0)\n");
    }
}

/* ========================================================================= */
/* JF — Force the gate's tag_push branch to fire by mirroring the dominant   */
/* WoW shape:  fld + fst + fld + fcomp + fstsw…                              */
/*                                                                           */
/* Setup: fld → push (td=1, tp=1).                                           */
/* Then fst (non-popping store) — doesn't touch tp/td.                       */
/*   Gate at fst's call: top_dirty branch fires (clears td, leaves tp).     */
/*   compile_run bails on FPR/GPR pressure.  Single-op for fst — doesn't    */
/*   touch tp.  After: td=0, tp=1.                                          */
/* Next op (here fld) hits gate's tag_push branch; flushes tag at slot=     */
/*   entry-TOP, falls through to compile_run, which bails on FCmp pressure  */
/*   (peak_gpr=9 vs pool=8).  Single-op handles fld.                        */
/*                                                                           */
/* Observable: tag word state via fnstenv after the sequence.                */
/* ========================================================================= */

static void test_jf_compare_chain(void) {
    volatile double a = 3.14, b = 2.71;
    volatile double sink;
    volatile uint16_t sw_eq, sw_lt, sw_gt;
    /* Repeat the pattern 6 times so threshold=3 and threshold=8 see           */
    /* different firing counts, increasing the chance of bug exposure.       */
    __asm__ volatile(
        /* 1 */
        "fldl %[a]\n\t"   /* push a (td=1, tp=1) */
        "fstpl %[s]\n\t"  /* pop+store; touches deferred state */
        "fldl %[a]\n\t"   /* push a */
        "fstl %[s]\n\t"   /* fst non-pop (td/tp untouched) */
        "fldl %[b]\n\t"   /* push b */
        "fcomp\n\t"       /* compare ST(0)=b vs ST(1)=a, pop */
        "fnstsw %%ax\n\t" /* read flags */
        "movw %%ax, %[sw_eq]\n\t"
        "fstpl %[s]\n\t" /* pop a */
        /* 2 — repeat with different ordering */
        "fldl %[b]\n\t"
        "fstpl %[s]\n\t"
        "fldl %[b]\n\t"
        "fstl %[s]\n\t"
        "fldl %[a]\n\t"
        "fcomp\n\t" /* a vs b */
        "fnstsw %%ax\n\t"
        "movw %%ax, %[sw_lt]\n\t"
        "fstpl %[s]\n\t"
        /* 3 — variant */
        "fldl %[a]\n\t"
        "fstpl %[s]\n\t"
        "fldl %[a]\n\t"
        "fstl %[s]\n\t"
        "fldl %[a]\n\t" /* same value */
        "fcomp\n\t"
        "fnstsw %%ax\n\t"
        "movw %%ax, %[sw_gt]\n\t"
        "fstpl %[s]\n\t"
        : [s] "=m"(sink), [sw_eq] "=m"(sw_eq), [sw_lt] "=m"(sw_lt), [sw_gt] "=m"(sw_gt)
        : [a] "m"(a), [b] "m"(b)
        : "ax", "st");
    /* a < b: C3=0 C2=0 C0=1 → bits[14:8] & 0x45 == 0x01 */
    /* But fcomp does ST(0) vs ST(1).  ST(0) was b, ST(1) was a:           */
    /* iteration 1: b(2.71) vs a(3.14) → b<a → C3=0,C2=0,C0=1 → 0x01      */
    /* iteration 2: a(3.14) vs b(2.71) → a>b → C3=0,C2=0,C0=0 → 0x00      */
    /* iteration 3: a vs a → equal → C3=1,C2=0,C0=0 → 0x40                */
    check_u16("JF: cmp1 (b<a → C0=1)", (sw_eq >> 8) & 0x45, 0x01);
    check_u16("JF: cmp2 (a>b → C0=0,C3=0)", (sw_lt >> 8) & 0x45, 0x00);
    check_u16("JF: cmp3 (a==a → C3=1)", (sw_gt >> 8) & 0x45, 0x40);
}

/* ========================================================================= */
/* JH — fcomp+fstp tag-word coherence (open bug from 2026-05-04 session).   */
/*                                                                           */
/* Shape (one iter):                                                         */
/*   fldl a; fstpl s;            push + pop                                 */
/*   fldl a; fstl  s;            push + non-pop store                       */
/*   fldl b; fcomp; fstpl s;     push, double-pop                            */
/*                                                                           */
/* Net stack effect per iter = 0.  Repeating across N iters with probes     */
/* between each iter must always read tag word = 0xffff (all 8 slots empty).*/
/* Native + X87_DISABLE_HOOK=1 give 0xffff; our JIT empirically gives       */
/* 0xc3ff (slots 5+6 spuriously kValid).                                    */
/* ========================================================================= */

static void test_jh_compare_chain_tagword(void) {
    volatile double a = 1.0, b = 2.0;
    volatile double sink;
    volatile uint8_t e1[28], e2[28];
    __asm__ volatile(
        /* Iter 1 */
        "fldl %[a]\n\t"
        "fstpl %[s]\n\t"
        "fldl %[a]\n\t"
        "fstl  %[s]\n\t"
        "fldl %[b]\n\t"
        "fcomp\n\t"
        "fstpl %[s]\n\t"
        "fnstenv %[e1]\n\t"
        /* Iter 2 */
        "fldl %[a]\n\t"
        "fstpl %[s]\n\t"
        "fldl %[a]\n\t"
        "fstl  %[s]\n\t"
        "fldl %[b]\n\t"
        "fcomp\n\t"
        "fstpl %[s]\n\t"
        "fnstenv %[e2]\n\t"
        : [s] "=m"(sink), [e1] "=m"(e1), [e2] "=m"(e2)
        : [a] "m"(a), [b] "m"(b)
        : "ax", "st");
    uint16_t tagw1, tagw2;
    memcpy(&tagw1, (const void*)&e1[8], 2);
    memcpy(&tagw2, (const void*)&e2[8], 2);
    check_u16("JH: tag_word after iter 1 (all empty)", tagw1, 0xffff);
    check_u16("JH: tag_word after iter 2 (all empty)", tagw2, 0xffff);
}

/* ========================================================================= */
/* J — Two consecutive runs separated by a non-x87 break.                    */
/* ========================================================================= */

static void test_j_two_runs(void) {
    volatile double a = 2.0, b = 3.0, c = 5.0, d = 7.0;
    volatile double r0, r1;
    volatile int dummy = 0;
    __asm__ volatile(
        /* Run 1 */
        "fldl %[a]\n\t"
        "fmull %[b]\n\t"
        "fldl %[c]\n\t"
        "fmull %[d]\n\t"
        "faddp\n\t"
        "fstpl %[r0]\n\t"
        /* Break */
        "movl $0, %[dummy]\n\t"
        /* Run 2 */
        "fldl %[a]\n\t"
        "fldl %[b]\n\t"
        "fmulp\n\t"
        "fstpl %[r1]\n"
        : [r0] "=m"(r0), [r1] "=m"(r1), [dummy] "+m"(dummy)
        : [a] "m"(a), [b] "m"(b), [c] "m"(c), [d] "m"(d));
    check_f64("J: two_runs r0 (2*3 + 5*7 = 41.0)", r0, 41.0);
    check_f64("J: two_runs r1 (2*3 = 6.0)", r1, 6.0);
}

int main(void) {
    test_a_mv_dot();
    test_b_mat3x3_mv();
    test_c_net_pop_1();
    test_d_net_push_1();
    test_e_tag_word_after_run();
    test_f_tag_word_balanced_run();
    test_g_fxam_after_gate();
    test_h_long_acc_chain();
    test_i_mixed_32bit_chain();
    test_jf_compare_chain();
    test_jh_compare_chain_tagword();
    test_j_two_runs();

    printf("\n%s: %d failure(s)\n", failures ? "FAILED" : "ALL PASSED", failures);
    return failures ? 1 : 0;
}
