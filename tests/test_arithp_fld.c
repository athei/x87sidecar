/*
 * test_arithp_fld.c — Tests for ARITHp ST(1) + FLD fusion (pop+push cancel).
 *
 * Covers: faddp+fld, fsubp+fld, fmulp+fld, fdivp+fld, fsubrp+fld, fdivrp+fld
 *         (m64 and m32), faddp+fldz, faddp+fld1, faddp+fild,
 *         faddp+fld ST(0), faddp+fld ST(1), stack depth preservation,
 *         consecutive pairs.
 *
 * Build: clang -arch x86_64 -O0 -g -o test_arithp_fld test_arithp_fld.c
 */
#include <math.h>
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

/* ==== faddp ST(1) + fld m64 ==== */
static void faddp_fld_m64(double* out_top, double* out_arith) {
    __asm__ volatile(
        "fldl %2\n"  /* ST(0) = 3.0 */
        "fldl %3\n"  /* ST(0) = 7.0, ST(1) = 3.0 */
        "faddp\n"    /* ST(0) = 3.0 + 7.0 = 10.0 */
        "fldl %4\n"  /* ST(0) = 42.0, ST(1) = 10.0 */
        "fstpl %0\n" /* store 42.0 */
        "fstpl %1\n" /* store 10.0 */
        : "=m"(*out_top), "=m"(*out_arith)
        : "m"((double){3.0}), "m"((double){7.0}), "m"((double){42.0}));
}

/* ==== fsubp ST(1) + fld m64 ==== */
/* GAS fsubp → Intel fsubrp (DE E0+i): ST(1) = ST(0) - ST(1) */
static void fsubp_fld_m64(double* out_top, double* out_arith) {
    __asm__ volatile(
        "fldl %2\n" /* ST(0) = 10.0 */
        "fldl %3\n" /* ST(0) = 3.0, ST(1) = 10.0 */
        "fsubp\n"   /* ST(0) = 3.0 - 10.0 = -7.0 */
        "fldl %4\n" /* ST(0) = 99.0, ST(1) = -7.0 */
        "fstpl %0\n"
        "fstpl %1\n"
        : "=m"(*out_top), "=m"(*out_arith)
        : "m"((double){10.0}), "m"((double){3.0}), "m"((double){99.0}));
}

/* ==== fmulp ST(1) + fld m64 ==== */
static void fmulp_fld_m64(double* out_top, double* out_arith) {
    __asm__ volatile(
        "fldl %2\n" /* ST(0) = 4.0 */
        "fldl %3\n" /* ST(0) = 5.0, ST(1) = 4.0 */
        "fmulp\n"   /* ST(0) = 4.0 * 5.0 = 20.0 */
        "fldl %4\n" /* ST(0) = 1.5, ST(1) = 20.0 */
        "fstpl %0\n"
        "fstpl %1\n"
        : "=m"(*out_top), "=m"(*out_arith)
        : "m"((double){4.0}), "m"((double){5.0}), "m"((double){1.5}));
}

/* ==== fdivp ST(1) + fld m64 ==== */
/* GAS fdivp → Intel fdivrp (DE F0+i): ST(1) = ST(0) / ST(1) */
static void fdivp_fld_m64(double* out_top, double* out_arith) {
    __asm__ volatile(
        "fldl %2\n" /* ST(0) = 20.0 */
        "fldl %3\n" /* ST(0) = 4.0, ST(1) = 20.0 */
        "fdivp\n"   /* ST(0) = 4.0 / 20.0 = 0.2 */
        "fldl %4\n" /* ST(0) = 7.0, ST(1) = 0.2 */
        "fstpl %0\n"
        "fstpl %1\n"
        : "=m"(*out_top), "=m"(*out_arith)
        : "m"((double){20.0}), "m"((double){4.0}), "m"((double){7.0}));
}

/* ==== fsubrp ST(1) + fld m64 ==== */
/* GAS fsubrp → Intel fsubp (DE E8+i): ST(1) = ST(1) - ST(0) */
static void fsubrp_fld_m64(double* out_top, double* out_arith) {
    __asm__ volatile(
        "fldl %2\n" /* ST(0) = 10.0 */
        "fldl %3\n" /* ST(0) = 3.0, ST(1) = 10.0 */
        "fsubrp\n"  /* ST(0) = 10.0 - 3.0 = 7.0 */
        "fldl %4\n" /* ST(0) = 55.0, ST(1) = 7.0 */
        "fstpl %0\n"
        "fstpl %1\n"
        : "=m"(*out_top), "=m"(*out_arith)
        : "m"((double){10.0}), "m"((double){3.0}), "m"((double){55.0}));
}

/* ==== fdivrp ST(1) + fld m64 ==== */
/* GAS fdivrp → Intel fdivp (DE F8+i): ST(1) = ST(1) / ST(0) */
static void fdivrp_fld_m64(double* out_top, double* out_arith) {
    __asm__ volatile(
        "fldl %2\n" /* ST(0) = 20.0 */
        "fldl %3\n" /* ST(0) = 4.0, ST(1) = 20.0 */
        "fdivrp\n"  /* ST(0) = 20.0 / 4.0 = 5.0 */
        "fldl %4\n" /* ST(0) = 11.0, ST(1) = 5.0 */
        "fstpl %0\n"
        "fstpl %1\n"
        : "=m"(*out_top), "=m"(*out_arith)
        : "m"((double){20.0}), "m"((double){4.0}), "m"((double){11.0}));
}

/* ==== faddp + fld m32 (f32 source) ==== */
static void faddp_fld_m32(double* out_top, double* out_arith) {
    float src_f32 = 2.5f;
    __asm__ volatile(
        "fldl %2\n" /* ST(0) = 1.0 */
        "fldl %3\n" /* ST(0) = 3.0, ST(1) = 1.0 */
        "faddp\n"   /* ST(0) = 4.0 */
        "flds %4\n" /* ST(0) = 2.5 (from f32), ST(1) = 4.0 */
        "fstpl %0\n"
        "fstpl %1\n"
        : "=m"(*out_top), "=m"(*out_arith)
        : "m"((double){1.0}), "m"((double){3.0}), "m"(src_f32));
}

/* ==== faddp + fldz ==== */
static void faddp_fldz(double* out_top, double* out_arith) {
    __asm__ volatile(
        "fldl %2\n" /* ST(0) = 5.0 */
        "fldl %3\n" /* ST(0) = 8.0, ST(1) = 5.0 */
        "faddp\n"   /* ST(0) = 13.0 */
        "fldz\n"    /* ST(0) = 0.0, ST(1) = 13.0 */
        "fstpl %0\n"
        "fstpl %1\n"
        : "=m"(*out_top), "=m"(*out_arith)
        : "m"((double){5.0}), "m"((double){8.0}));
}

/* ==== faddp + fld1 ==== */
static void faddp_fld1(double* out_top, double* out_arith) {
    __asm__ volatile(
        "fldl %2\n" /* ST(0) = 5.0 */
        "fldl %3\n" /* ST(0) = 8.0, ST(1) = 5.0 */
        "faddp\n"   /* ST(0) = 13.0 */
        "fld1\n"    /* ST(0) = 1.0, ST(1) = 13.0 */
        "fstpl %0\n"
        "fstpl %1\n"
        : "=m"(*out_top), "=m"(*out_arith)
        : "m"((double){5.0}), "m"((double){8.0}));
}

/* ==== faddp + fild m32 ==== */
static void faddp_fild(double* out_top, double* out_arith) {
    int32_t ival = 123;
    __asm__ volatile(
        "fldl %2\n"  /* ST(0) = 2.0 */
        "fldl %3\n"  /* ST(0) = 6.0, ST(1) = 2.0 */
        "faddp\n"    /* ST(0) = 8.0 */
        "fildl %4\n" /* ST(0) = 123.0, ST(1) = 8.0 */
        "fstpl %0\n"
        "fstpl %1\n"
        : "=m"(*out_top), "=m"(*out_arith)
        : "m"((double){2.0}), "m"((double){6.0}), "m"(ival));
}

/* ==== faddp + fld ST(0): reads arithmetic result ==== */
static void faddp_fld_st0(double* out_top, double* out_next) {
    __asm__ volatile(
        "fldl %2\n"     /* ST(0) = 3.0 */
        "fldl %3\n"     /* ST(0) = 7.0, ST(1) = 3.0 */
        "faddp\n"       /* ST(0) = 10.0 */
        "fld %%st(0)\n" /* ST(0) = 10.0 (copy), ST(1) = 10.0 */
        "fstpl %0\n"    /* store 10.0 */
        "fstpl %1\n"    /* store 10.0 */
        : "=m"(*out_top), "=m"(*out_next)
        : "m"((double){3.0}), "m"((double){7.0}));
}

/* ==== faddp + fld ST(1): reads deeper register (old_ST(2)) ==== */
static void faddp_fld_st1(double* out_top, double* out_next) {
    __asm__ volatile(
        "fldl %2\n"      /* push 100.0: ST(0)=100 */
        "fldl %3\n"      /* push 3.0:  ST(0)=3, ST(1)=100 */
        "fldl %4\n"      /* push 7.0:  ST(0)=7, ST(1)=3, ST(2)=100 */
        "faddp\n"        /* ST(0) = 3+7 = 10, ST(1) = 100 */
        "fld %%st(1)\n"  /* ST(0) = 100, ST(1) = 10, ST(2) = 100 */
        "fstpl %0\n"     /* store 100 */
        "fstpl %1\n"     /* store 10 */
        "fstp %%st(0)\n" /* discard 100 */
        : "=m"(*out_top), "=m"(*out_next)
        : "m"((double){100.0}), "m"((double){3.0}), "m"((double){7.0}));
}

/* ==== Stack depth preserved: arithp+fld with deeper values surviving ==== */
static void arithp_fld_depth(double* out_top, double* out_arith, double* out_deep) {
    __asm__ volatile(
        "fldl %3\n"  /* push 100.0: ST(0)=100 */
        "fldl %4\n"  /* push 10.0:  ST(0)=10, ST(1)=100 */
        "fldl %5\n"  /* push 3.0:   ST(0)=3, ST(1)=10, ST(2)=100 */
        "faddp\n"    /* ST(0) = 10+3 = 13, ST(1) = 100 */
        "fldl %6\n"  /* ST(0) = 42.0, ST(1) = 13, ST(2) = 100 */
        "fstpl %0\n" /* store 42.0 */
        "fstpl %1\n" /* store 13.0 */
        "fstpl %2\n" /* store 100.0 */
        : "=m"(*out_top), "=m"(*out_arith), "=m"(*out_deep)
        : "m"((double){100.0}), "m"((double){10.0}), "m"((double){3.0}), "m"((double){42.0}));
}

/* ==== Consecutive arithp+fld pairs ==== */
static void consecutive_pairs(double* out0, double* out1) {
    __asm__ volatile(
        "fldl %2\n"  /* push 2.0 */
        "fldl %3\n"  /* push 3.0: ST(0)=3, ST(1)=2 */
        "fmulp\n"    /* ST(0) = 6 */
        "fldl %4\n"  /* ST(0) = 10.0, ST(1) = 6 — first arithp+fld pair */
        "faddp\n"    /* ST(0) = 6 + 10 = 16 */
        "fldl %5\n"  /* ST(0) = 100.0, ST(1) = 16 — second arithp+fld pair */
        "fstpl %0\n" /* store 100.0 */
        "fstpl %1\n" /* store 16.0 */
        : "=m"(*out0), "=m"(*out1)
        : "m"((double){2.0}), "m"((double){3.0}), "m"((double){10.0}), "m"((double){100.0}));
}

int main(void) {
    /* faddp + fld m64 */
    {
        double top, arith;
        faddp_fld_m64(&top, &arith);
        check("faddp+fld_m64: FLD value (42)", top, 42.0);
        check("faddp+fld_m64: arith result (3+7=10)", arith, 10.0);
    }

    /* fsubp + fld m64 (GAS fsubp → ST(0)-ST(1)) */
    {
        double top, arith;
        fsubp_fld_m64(&top, &arith);
        check("fsubp+fld_m64: FLD value (99)", top, 99.0);
        check("fsubp+fld_m64: arith result (3-10=-7)", arith, -7.0);
    }

    /* fmulp + fld m64 */
    {
        double top, arith;
        fmulp_fld_m64(&top, &arith);
        check("fmulp+fld_m64: FLD value (1.5)", top, 1.5);
        check("fmulp+fld_m64: arith result (4*5=20)", arith, 20.0);
    }

    /* fdivp + fld m64 (GAS fdivp → ST(0)/ST(1)) */
    {
        double top, arith;
        fdivp_fld_m64(&top, &arith);
        check("fdivp+fld_m64: FLD value (7)", top, 7.0);
        check("fdivp+fld_m64: arith result (4/20=0.2)", arith, 0.2);
    }

    /* fsubrp + fld m64 (GAS fsubrp → ST(1)-ST(0)) */
    {
        double top, arith;
        fsubrp_fld_m64(&top, &arith);
        check("fsubrp+fld_m64: FLD value (55)", top, 55.0);
        check("fsubrp+fld_m64: arith result (10-3=7)", arith, 7.0);
    }

    /* fdivrp + fld m64 (GAS fdivrp → ST(1)/ST(0)) */
    {
        double top, arith;
        fdivrp_fld_m64(&top, &arith);
        check("fdivrp+fld_m64: FLD value (11)", top, 11.0);
        check("fdivrp+fld_m64: arith result (20/4=5)", arith, 5.0);
    }

    /* faddp + fld m32 (f32 source, promoted to f64) */
    {
        double top, arith;
        faddp_fld_m32(&top, &arith);
        check("faddp+fld_m32: FLD value (2.5)", top, 2.5);
        check("faddp+fld_m32: arith result (1+3=4)", arith, 4.0);
    }

    /* faddp + fldz */
    {
        double top, arith;
        faddp_fldz(&top, &arith);
        check("faddp+fldz: FLD value (0)", top, 0.0);
        check("faddp+fldz: arith result (5+8=13)", arith, 13.0);
    }

    /* faddp + fld1 */
    {
        double top, arith;
        faddp_fld1(&top, &arith);
        check("faddp+fld1: FLD value (1)", top, 1.0);
        check("faddp+fld1: arith result (5+8=13)", arith, 13.0);
    }

    /* faddp + fild m32 */
    {
        double top, arith;
        faddp_fild(&top, &arith);
        check("faddp+fild: FLD value (123)", top, 123.0);
        check("faddp+fild: arith result (2+6=8)", arith, 8.0);
    }

    /* faddp + fld ST(0): reads arithmetic result */
    {
        double top, next;
        faddp_fld_st0(&top, &next);
        check("faddp+fld_st0: FLD value (copy of 10)", top, 10.0);
        check("faddp+fld_st0: arith result (3+7=10)", next, 10.0);
    }

    /* faddp + fld ST(1): reads deeper register (old_ST(2) = 100) */
    {
        double top, next;
        faddp_fld_st1(&top, &next);
        check("faddp+fld_st1: FLD value (old depth 2 = 100)", top, 100.0);
        check("faddp+fld_st1: arith result (3+7=10)", next, 10.0);
    }

    /* Stack depth preserved */
    {
        double top, arith, deep;
        arithp_fld_depth(&top, &arith, &deep);
        check("depth: FLD value (42)", top, 42.0);
        check("depth: arith result (10+3=13)", arith, 13.0);
        check("depth: remaining deep (100)", deep, 100.0);
    }

    /* Consecutive pairs */
    {
        double a, b;
        consecutive_pairs(&a, &b);
        check("consecutive: FLD value (100)", a, 100.0);
        check("consecutive: accumulated (6+10=16)", b, 16.0);
    }

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
