/*
 * test_fstp_arith_fstp.c — Tests for FSTP mem + non-popping ARITH mem +
 * FSTP mem 3-op fusion.
 *
 * Covers: fadd/fsub/fsubr/fmul/fdiv/fdivr × {m32, m64} × {non-aliased, aliased
 *         dest1==arith.src}.  The aliasing case checks that the f32 truncation
 *         from the first FSTP is observed by the arith's memory load (matches
 *         x86 program-order memory semantics).
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void check(const char* name, double got, double expected) {
    uint64_t a;
    uint64_t b;
    memcpy(&a, &got, 8);
    memcpy(&b, &expected, 8);
    if (a != b) {
        printf("FAIL  %-65s  got=%.17g  expected=%.17g\n", name, got, expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

static void check_f32(const char* name, float got, float expected) {
    uint32_t a;
    uint32_t b;
    memcpy(&a, &got, 4);
    memcpy(&b, &expected, 4);
    if (a != b) {
        printf("FAIL  %-65s  got=%.9g  expected=%.9g\n", name, (double)got, (double)expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

/* === fstp m64 + fadd m64 + fstp m64 ===
 *   stack: ST(0)=A, ST(1)=B
 *   [d1] = A; pop;  ST(0) = B + src;  [d2] = B+src; pop
 */
static void run_m64_fadd(double a, double b, double src, double* d1, double* d2) {
    __asm__ volatile(
        "fldl %3\n"  /* push B → ST(0)=B */
        "fldl %2\n"  /* push A → ST(0)=A, ST(1)=B */
        "fstpl %0\n" /* [d1] = A, pop  → ST(0)=B */
        "faddl %4\n" /* ST(0) = B + src */
        "fstpl %1\n" /* [d2] = B+src, pop */
        : "=m"(*d1), "=m"(*d2)
        : "m"(a), "m"(b), "m"(src)
        : "memory");
}

/* === fstp m32 + fadd m32 + fstp m32 (the actual workload pattern) === */
static void run_m32_fadd(double a, double b, float src, float* d1, float* d2) {
    __asm__ volatile(
        "fldl %3\n"
        "fldl %2\n"
        "fstps %0\n"
        "fadds %4\n"
        "fstps %1\n"
        : "=m"(*d1), "=m"(*d2)
        : "m"(a), "m"(b), "m"(src)
        : "memory");
}

/* === fstp m32 + fmul m32 + fstp m32 === */
static void run_m32_fmul(double a, double b, float src, float* d1, float* d2) {
    __asm__ volatile(
        "fldl %3\n"
        "fldl %2\n"
        "fstps %0\n"
        "fmuls %4\n"
        "fstps %1\n"
        : "=m"(*d1), "=m"(*d2)
        : "m"(a), "m"(b), "m"(src)
        : "memory");
}

/* === fstp m32 + fsub m32 + fstp m32 ===
 *   ST(0) = B - src
 */
static void run_m32_fsub(double a, double b, float src, float* d1, float* d2) {
    __asm__ volatile(
        "fldl %3\n"
        "fldl %2\n"
        "fstps %0\n"
        "fsubs %4\n"
        "fstps %1\n"
        : "=m"(*d1), "=m"(*d2)
        : "m"(a), "m"(b), "m"(src)
        : "memory");
}

/* === fstp m32 + fsubr m32 + fstp m32 ===
 *   ST(0) = src - B
 */
static void run_m32_fsubr(double a, double b, float src, float* d1, float* d2) {
    __asm__ volatile(
        "fldl %3\n"
        "fldl %2\n"
        "fstps %0\n"
        "fsubrs %4\n"
        "fstps %1\n"
        : "=m"(*d1), "=m"(*d2)
        : "m"(a), "m"(b), "m"(src)
        : "memory");
}

/* === fstp m32 + fdiv m32 + fstp m32 === */
static void run_m32_fdiv(double a, double b, float src, float* d1, float* d2) {
    __asm__ volatile(
        "fldl %3\n"
        "fldl %2\n"
        "fstps %0\n"
        "fdivs %4\n"
        "fstps %1\n"
        : "=m"(*d1), "=m"(*d2)
        : "m"(a), "m"(b), "m"(src)
        : "memory");
}

/* === Aliased case: dest1 == arith.src, m32 ===
 *   1. [shared] = (f32)A
 *   2. ST(0) = B + (f32_load)[shared] = B + (f32)A
 *   3. [d2] = B + (f32)A
 *
 * If our fused emit reordered the load before the store, the load would
 * read the original [shared] value and ST(0) would equal B + original_shared,
 * which differs from B + (f32)A.  This test pins down the program order.
 */
static void run_m32_fadd_aliased(double a, double b, float* shared, float* d2) {
    /* Pre-fill shared with a sentinel so any reordering bug is observable. */
    *shared = 1.0e10f;
    __asm__ volatile(
        "fldl %2\n"
        "fldl %1\n"
        "fstps %0\n" /* [shared] = (f32)A */
        "fadds %0\n" /* ST(0) = B + (f32)A */
        "fstps %3\n"
        : "=m"(*shared), "+m"(a), "+m"(b)
        : "m"(*d2)
        : "memory");
    (void)d2;  /* d2 written via inline asm above */
}

/* Same aliased shape but using *d2 as the actual destination address. */
static void run_m32_fadd_aliased_real(double a, double b, float* shared, float* d2) {
    *shared = 1.0e10f;
    __asm__ volatile(
        "fldl %3\n" /* push B */
        "fldl %2\n" /* push A */
        "fstps %0\n"
        "fadds %0\n"
        "fstps %1\n"
        : "=m"(*shared), "=m"(*d2)
        : "m"(a), "m"(b)
        : "memory");
}

int main(void) {
    printf("=== fstp + arith + fstp (m64, fadd) ===\n");
    {
        double d1;
        double d2;
        run_m64_fadd(3.0, 4.0, 10.0, &d1, &d2);
        check("[d1]=3.0", d1, 3.0);
        check("[d2]=4.0+10.0=14.0", d2, 14.0);
    }

    printf("\n=== fstp + arith + fstp (m32, fadd) — workload pattern ===\n");
    {
        float d1;
        float d2;
        run_m32_fadd(3.5, 4.25, 0.5f, &d1, &d2);
        check_f32("[d1]=3.5f", d1, 3.5f);
        check_f32("[d2]=4.25f+0.5f=4.75f", d2, 4.75f);
    }

    printf("\n=== fstp + arith + fstp (m32, fmul) ===\n");
    {
        float d1;
        float d2;
        run_m32_fmul(2.5, 3.0, 4.0f, &d1, &d2);
        check_f32("[d1]=2.5f", d1, 2.5f);
        check_f32("[d2]=3.0f*4.0f=12.0f", d2, 12.0f);
    }

    printf("\n=== fstp + arith + fstp (m32, fsub) ===\n");
    {
        float d1;
        float d2;
        run_m32_fsub(3.0, 10.0, 4.0f, &d1, &d2);
        check_f32("[d1]=3.0f", d1, 3.0f);
        check_f32("[d2]=10.0f-4.0f=6.0f", d2, 6.0f);
    }

    printf("\n=== fstp + arith + fstp (m32, fsubr) ===\n");
    {
        float d1;
        float d2;
        run_m32_fsubr(3.0, 4.0, 10.0f, &d1, &d2);
        check_f32("[d1]=3.0f", d1, 3.0f);
        check_f32("[d2]=10.0f-4.0f=6.0f", d2, 6.0f);
    }

    printf("\n=== fstp + arith + fstp (m32, fdiv) ===\n");
    {
        float d1;
        float d2;
        run_m32_fdiv(3.0, 12.0, 4.0f, &d1, &d2);
        check_f32("[d1]=3.0f", d1, 3.0f);
        check_f32("[d2]=12.0f/4.0f=3.0f", d2, 3.0f);
    }

    printf("\n=== Aliased: dest1 == arith.src (m32, fadd) ===\n");
    /* A is chosen so f32-truncation differs from f64: 1.0/3.0 isn't exact. */
    {
        float shared = 0.0f;
        float d2 = 0.0f;
        const double a = 1.0 / 3.0;
        const double b = 1.0;
        run_m32_fadd_aliased_real(a, b, &shared, &d2);
        const float a_as_f32 = (float)a;
        const float expected_d2 = (float)(b + (double)a_as_f32);
        check_f32("aliased [shared]=(f32)(1/3)", shared, a_as_f32);
        check_f32("aliased [d2]=B + (f32)(1/3)", d2, expected_d2);
    }

    printf("\n%s  (%d failure%s)\n", failures == 0 ? "ALL PASS" : "SOME FAILURES", failures,
           failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
