/*
 * test_fxch.c — Tests for FXCH register exchange, including OPT-G renaming.
 *
 * Covers: FXCH ST(0) (no-op), FXCH ST(1)..ST(7), consecutive FXCHs,
 *         FXCH + arithmetic, FXCH + FSTP, FXCH in loops.
 *
 * Build: clang -arch x86_64 -O0 -g -o test_fxch test_fxch.c
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

/* ==== FXCH ST(0) — no-op ==== */
static double fxch_st0_noop(void) {
    double r;
    __asm__ volatile(
        "fldl %1\n"
        "fxch %%st(0)\n" /* no-op */
        "fstpl %0\n"
        : "=m"(r)
        : "m"((double){3.14}));
    return r;
}

/* ==== FXCH ST(1) — basic swap ==== */
static void fxch_st1_basic(double* out0, double* out1) {
    __asm__ volatile(
        "fldl %2\n"      /* ST(0)=2.0 */
        "fldl %3\n"      /* ST(0)=5.0, ST(1)=2.0 */
        "fxch %%st(1)\n" /* ST(0)=2.0, ST(1)=5.0 */
        "fstpl %0\n"     /* store ST(0)=2.0, ST(0)=5.0 */
        "fstpl %1\n"     /* store ST(0)=5.0 */
        : "=m"(*out0), "=m"(*out1)
        : "m"((double){2.0}), "m"((double){5.0}));
}

/* ==== FXCH ST(2) — deeper swap ==== */
static void fxch_st2(double* out0, double* out1, double* out2) {
    __asm__ volatile(
        "fldl %3\n"      /* push 1.0 */
        "fldl %4\n"      /* push 2.0 */
        "fldl %5\n"      /* push 3.0: ST(0)=3, ST(1)=2, ST(2)=1 */
        "fxch %%st(2)\n" /* ST(0)=1, ST(1)=2, ST(2)=3 */
        "fstpl %0\n"     /* pop 1.0 */
        "fstpl %1\n"     /* pop 2.0 */
        "fstpl %2\n"     /* pop 3.0 */
        : "=m"(*out0), "=m"(*out1), "=m"(*out2)
        : "m"((double){1.0}), "m"((double){2.0}), "m"((double){3.0}));
}

/* ==== FXCH ST(3) — even deeper ==== */
static void fxch_st3(double* out0, double* out3) {
    __asm__ volatile(
        "fldl %2\n"      /* push 10 */
        "fldl %3\n"      /* push 20 */
        "fldl %4\n"      /* push 30 */
        "fldl %5\n"      /* push 40: ST(0)=40, ST(1)=30, ST(2)=20, ST(3)=10 */
        "fxch %%st(3)\n" /* ST(0)=10, ST(1)=30, ST(2)=20, ST(3)=40 */
        "fstpl %0\n"     /* pop ST(0)=10 */
        "fstp %%st(0)\n" /* discard 30 */
        "fstp %%st(0)\n" /* discard 20 */
        "fstpl %1\n"     /* pop ST(0)=40 */
        : "=m"(*out0), "=m"(*out3)
        : "m"((double){10.0}), "m"((double){20.0}), "m"((double){30.0}), "m"((double){40.0}));
}

/* ==== Consecutive FXCHs: FXCH ST(1); FXCH ST(2); FXCH ST(1) ==== */
static void fxch_consecutive(double* out0, double* out1, double* out2) {
    __asm__ volatile(
        "fldl %3\n"      /* push A=1 */
        "fldl %4\n"      /* push B=2 */
        "fldl %5\n"      /* push C=3: ST(0)=C, ST(1)=B, ST(2)=A */
        "fxch %%st(1)\n" /* ST(0)=B, ST(1)=C, ST(2)=A */
        "fxch %%st(2)\n" /* ST(0)=A, ST(1)=C, ST(2)=B */
        "fxch %%st(1)\n" /* ST(0)=C, ST(1)=A, ST(2)=B */
        "fstpl %0\n"     /* pop C=3 */
        "fstpl %1\n"     /* pop A=1 */
        "fstpl %2\n"     /* pop B=2 */
        : "=m"(*out0), "=m"(*out1), "=m"(*out2)
        : "m"((double){1.0}), "m"((double){2.0}), "m"((double){3.0}));
}

/* ==== FXCH + FADDP ==== */
static double fxch_faddp(void) {
    double r;
    __asm__ volatile(
        "fldl %1\n"      /* ST(0)=3 */
        "fldl %2\n"      /* ST(0)=7, ST(1)=3 */
        "fxch %%st(1)\n" /* ST(0)=3, ST(1)=7 */
        "faddp\n"        /* ST(0) = 7+3 = 10 */
        "fstpl %0\n"
        : "=m"(r)
        : "m"((double){3.0}), "m"((double){7.0}));
    return r;
}

/* ==== FXCH + FSUBP (non-commutative: tests operand order) ==== */
static double fxch_fsubp(void) {
    double r;
    /* Push 10, push 3 → ST(0)=3, ST(1)=10
     * FXCH → ST(0)=10, ST(1)=3
     * GAS fsubp = Intel fsubrp for popping reg forms: ST(0) - ST(1) = 10 - 3 = 7 */
    __asm__ volatile(
        "fldl %1\n"      /* ST(0)=10 */
        "fldl %2\n"      /* ST(0)=3, ST(1)=10 */
        "fxch %%st(1)\n" /* ST(0)=10, ST(1)=3 */
        "fsubp\n"        /* GAS fsubp = Intel fsubrp: ST(0) - ST(1) = 10 - 3 = 7 */
        "fstpl %0\n"
        : "=m"(r)
        : "m"((double){10.0}), "m"((double){3.0}));
    return r;
}

/* ==== FXCH + FMULP ==== */
static double fxch_fmulp(void) {
    double r;
    __asm__ volatile(
        "fldl %1\n"      /* ST(0)=4 */
        "fldl %2\n"      /* ST(0)=5, ST(1)=4 */
        "fxch %%st(1)\n" /* ST(0)=4, ST(1)=5 */
        "fmulp\n"        /* ST(0) = 5*4 = 20 */
        "fstpl %0\n"
        : "=m"(r)
        : "m"((double){4.0}), "m"((double){5.0}));
    return r;
}

/* ==== FXCH + FSTP ==== */
static double fxch_fstp(void) {
    double r;
    __asm__ volatile(
        "fldl %1\n"      /* ST(0)=100 */
        "fldl %2\n"      /* ST(0)=200, ST(1)=100 */
        "fxch %%st(1)\n" /* ST(0)=100, ST(1)=200 */
        "fstp %%st(0)\n" /* discard ST(0)=100, now ST(0)=200 */
        "fstpl %0\n"
        : "=m"(r)
        : "m"((double){100.0}), "m"((double){200.0}));
    return r;
}

/* ==== FXCH in a loop ==== */
static double fxch_loop(void) {
    double r;
    /* Accumulate sum via repeated FXCH + FADDP pattern */
    __asm__ volatile(
        "fldz\n"         /* accumulator = 0 */
        "fld1\n"         /* ST(0)=1, ST(1)=0 */
        "fxch %%st(1)\n" /* ST(0)=0, ST(1)=1 */
        "faddp\n"        /* ST(0) = 1 */
        "fld1\n"
        "fxch %%st(1)\n"
        "faddp\n" /* ST(0) = 2 */
        "fld1\n"
        "fxch %%st(1)\n"
        "faddp\n" /* ST(0) = 3 */
        "fld1\n"
        "fxch %%st(1)\n"
        "faddp\n" /* ST(0) = 4 */
        "fld1\n"
        "fxch %%st(1)\n"
        "faddp\n" /* ST(0) = 5 */
        "fstpl %0\n"
        : "=m"(r));
    return r;
}

/* ==== FXCH double swap (swap back = no-op) ==== */
static double fxch_double_swap(void) {
    double r;
    __asm__ volatile(
        "fldl %1\n"      /* ST(0)=42 */
        "fldl %2\n"      /* ST(0)=99, ST(1)=42 */
        "fxch %%st(1)\n" /* ST(0)=42, ST(1)=99 */
        "fxch %%st(1)\n" /* ST(0)=99, ST(1)=42 (back to original) */
        "fstpl %0\n"     /* should be 99 */
        "fstp %%st(0)\n"
        : "=m"(r)
        : "m"((double){42.0}), "m"((double){99.0}));
    return r;
}

/* ==== FXCH + FCOM (compare after exchange) ==== */
static void fxch_fcom(double* out_sw) {
    uint16_t sw;
    __asm__ volatile(
        "fldl %1\n"      /* ST(0)=5.0 */
        "fldl %2\n"      /* ST(0)=3.0, ST(1)=5.0 */
        "fxch %%st(1)\n" /* ST(0)=5.0, ST(1)=3.0 */
        "fcom %%st(1)\n" /* compare ST(0)=5 vs ST(1)=3 → GT */
        "fnstsw %%ax\n"
        "movw %%ax, %0\n"
        "fstp %%st(0)\n"
        "fstp %%st(0)\n"
        : "=m"(sw)
        : "m"((double){5.0}), "m"((double){3.0})
        : "ax");
    /* GT: C3=0, C2=0, C0=0 → bits 14,10,8 all clear */
    /* Mask out TOP bits and C1 to check only condition codes */
    uint16_t cc = sw & ((1 << 14) | (1 << 10) | (1 << 8));
    *out_sw = (double)cc;
}

int main(void) {
    /* FXCH ST(0) no-op */
    check("fxch_st0_noop", fxch_st0_noop(), 3.14);

    /* FXCH ST(1) basic */
    {
        double a, b;
        fxch_st1_basic(&a, &b);
        check("fxch_st1_basic: ST(0) after swap", a, 2.0);
        check("fxch_st1_basic: ST(1) after swap", b, 5.0);
    }

    /* FXCH ST(2) */
    {
        double a, b, c;
        fxch_st2(&a, &b, &c);
        check("fxch_st2: ST(0)", a, 1.0);
        check("fxch_st2: ST(1)", b, 2.0);
        check("fxch_st2: ST(2)", c, 3.0);
    }

    /* FXCH ST(3) */
    {
        double a, d;
        fxch_st3(&a, &d);
        check("fxch_st3: ST(0) after swap", a, 10.0);
        check("fxch_st3: ST(3) after swap", d, 40.0);
    }

    /* Consecutive FXCHs */
    {
        double a, b, c;
        fxch_consecutive(&a, &b, &c);
        check("fxch_consecutive: ST(0)", a, 3.0);
        check("fxch_consecutive: ST(1)", b, 1.0);
        check("fxch_consecutive: ST(2)", c, 2.0);
    }

    /* FXCH + arithmetic */
    check("fxch_faddp", fxch_faddp(), 10.0);
    check("fxch_fsubp", fxch_fsubp(), 7.0);
    check("fxch_fmulp", fxch_fmulp(), 20.0);

    /* FXCH + FSTP */
    check("fxch_fstp", fxch_fstp(), 200.0);

    /* FXCH in loop */
    check("fxch_loop (5 iterations)", fxch_loop(), 5.0);

    /* FXCH double swap (no-op) */
    check("fxch_double_swap", fxch_double_swap(), 99.0);

    /* FXCH + FCOM */
    {
        double cc;
        fxch_fcom(&cc);
        check("fxch_fcom: GT condition codes", cc, 0.0);
    }

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
