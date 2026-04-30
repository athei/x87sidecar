/*
 * test_fptan.c — FPTAN routed through sidecar IPC + libm.
 *
 * x86 fptan: ST(0) := tan(ST(0)), push 1.0.  After: ST(0)=1.0, ST(1)=tan(v).
 *
 * Inline asm: fld v; fptan; fstpl one; fstpl tan
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

static void do_fptan(double v, double *out_tan, double *out_one) {
    __asm__ volatile(
        "fldl  %2\n\t"
        "fptan\n\t"
        "fstpl %1\n\t"   /* 1.0 at ST(0) → out_one */
        "fstpl %0\n\t"   /* tan at new ST(0) → out_tan */
        : "=m"(*out_tan), "=m"(*out_one)
        : "m"(v)
        : "st");
}

int main(void) {
    /* Pre-computed expected bit patterns; avoids confounding the test
       with libm's own fsin/fcos calls under JIT.  Only tan(0)=0 picks
       a value where x87 80-bit fptan and libm tan agree exactly; for
       general inputs the two paths' low-bit results diverge.  This
       test still covers the 2-output (tan + push 1.0) JIT-emit shape. */
    double t, one;

    do_fptan(0.0, &t, &one);
    check_bitexact_u64("fptan(0).tan",     t,   0x0000000000000000ULL);
    check_bitexact_u64("fptan(0).one",     one, 0x3ff0000000000000ULL);

    do_fptan(-0.0, &t, &one);
    check_bitexact_u64("fptan(-0).tan",    t,   0x8000000000000000ULL);
    check_bitexact_u64("fptan(-0).one",    one, 0x3ff0000000000000ULL);

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
