/*
 * test_fprem.c — FPREM routed through sidecar IPC + libm.
 *
 * x86 fprem: ST(0) := fmod(ST(0), ST(1)).  No pop.  Real x87 is
 * iterative (sets C2=1 if reduction incomplete); our simplification
 * always completes in one libm fmod call.  Sidecar logs a one-shot
 * warning on first call.
 *
 * Inline asm: fld y; fld x; fprem; fstpl r; ffree st(0); fincstp
 * (the second fld leaves y in ST(1); fprem leaves x % y in ST(0);
 * we then drop ST(1)=y to leave the stack empty).
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static int check_bitexact(const char *name, double got, double expected) {
    uint64_t g, e;
    memcpy(&g, &got, sizeof(g));
    memcpy(&e, &expected, sizeof(e));
    if (g == e) {
        printf("PASS  %-40s  got=0x%016llx (%.17g)\n", name,
               (unsigned long long)g, got);
        return 1;
    }
    printf("FAIL  %-40s  got=0x%016llx (%.17g)  expected=0x%016llx (%.17g)\n",
           name, (unsigned long long)g, got, (unsigned long long)e, expected);
    failures++;
    return 0;
}

static int check_bitexact_u64(const char *name, double got, uint64_t expected_bits) {
    double expected;
    memcpy(&expected, &expected_bits, sizeof(expected));
    return check_bitexact(name, got, expected);
}

static double do_fprem(double x, double y) {
    double r;
    __asm__ volatile(
        "fldl  %1\n\t"   /* ST(0) = y */
        "fldl  %2\n\t"   /* ST(0) = x; ST(1) = y */
        "fprem\n\t"      /* ST(0) = fmod(x, y) */
        "fstpl %0\n\t"   /* pop result */
        "ffree %%st(0)\n\t"  /* drop leftover y */
        "fincstp\n\t"
        : "=m"(r)
        : "m"(y), "m"(x)
        : "st");
    return r;
}

int main(void) {
    /* Inputs where single-shot fmod and iterative x87 fprem agree —
       ratios with |x/y| <= 2^64 (all our cases) AND where the reduced
       value is the same. */
    check_bitexact_u64("fprem(5, 3)",   do_fprem(5.0, 3.0),   0x4000000000000000ULL); /* 2.0 */
    check_bitexact_u64("fprem(7, 3)",   do_fprem(7.0, 3.0),   0x3ff0000000000000ULL); /* 1.0 */
    check_bitexact_u64("fprem(8, 3)",   do_fprem(8.0, 3.0),   0x4000000000000000ULL); /* 2.0 */
    check_bitexact_u64("fprem(1, 1)",   do_fprem(1.0, 1.0),   0x0000000000000000ULL); /* 0.0 */
    check_bitexact_u64("fprem(-5, 3)",  do_fprem(-5.0, 3.0),  0xc000000000000000ULL); /* -2.0 */

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
