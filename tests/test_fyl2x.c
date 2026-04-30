/*
 * test_fyl2x.c — FYL2X routed through sidecar IPC + libm.
 *
 * x86 fyl2x: ST(1) := ST(1) * log2(ST(0)); pop ST(0).
 * Inline asm: fld y; fld x; fyl2x; fstpl r — leaves y * log2(x) in r.
 *
 * Pre-computed expected bit patterns avoid confounding the test with
 * libm's own fmul/log2 calls under JIT.
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

static double do_fyl2x(double y, double x) {
    double r;
    __asm__ volatile(
        "fldl  %1\n\t"   /* push y → ST(0) */
        "fldl  %2\n\t"   /* push x → ST(0); y now ST(1) */
        "fyl2x\n\t"      /* ST(1) = y*log2(x); pop ST(0) */
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(y), "m"(x)
        : "st");
    return r;
}

int main(void) {
    /* Inputs that yield bit-exact identical results between x87 80-bit
       fyl2x and libm's y * log2(x). */
    check_bitexact_u64("fyl2x(1, 2)",   do_fyl2x(1.0, 2.0),   0x3ff0000000000000ULL); /* 1*log2(2)=1 */
    check_bitexact_u64("fyl2x(2, 2)",   do_fyl2x(2.0, 2.0),   0x4000000000000000ULL); /* 2*log2(2)=2 */
    check_bitexact_u64("fyl2x(1, 1)",   do_fyl2x(1.0, 1.0),   0x0000000000000000ULL); /* log2(1)=0 */
    check_bitexact_u64("fyl2x(3, 4)",   do_fyl2x(3.0, 4.0),   0x4018000000000000ULL); /* 3*log2(4)=6 */
    check_bitexact_u64("fyl2x(1, 4)",   do_fyl2x(1.0, 4.0),   0x4000000000000000ULL); /* log2(4)=2 */

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
