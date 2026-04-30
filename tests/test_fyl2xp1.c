/*
 * test_fyl2xp1.c — FYL2XP1 routed through sidecar IPC + libm.
 *
 * x86 fyl2xp1: ST(1) := ST(1) * log2(ST(0) + 1.0); pop ST(0).
 * Domain: -1+1/sqrt(2) <= ST(0) <= 1-1/sqrt(2) for full x87 precision,
 * but we always compute via libm regardless.
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

static double do_fyl2xp1(double y, double x) {
    double r;
    __asm__ volatile(
        "fldl  %1\n\t"   /* push y → ST(0) */
        "fldl  %2\n\t"   /* push x → ST(0); y now ST(1) */
        "fyl2xp1\n\t"    /* ST(1) = y*log2(x+1); pop */
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(y), "m"(x)
        : "st");
    return r;
}

int main(void) {
    /* Inputs that give bit-exact results between x87 80-bit fyl2xp1
       and libm's y * log2(x+1). */
    check_bitexact_u64("fyl2xp1(1, 0)",   do_fyl2xp1(1.0, 0.0),   0x0000000000000000ULL); /* log2(1)=0 */
    check_bitexact_u64("fyl2xp1(2, 0)",   do_fyl2xp1(2.0, 0.0),   0x0000000000000000ULL); /* 2*log2(1)=0 */
    check_bitexact_u64("fyl2xp1(1, 1)",   do_fyl2xp1(1.0, 1.0),   0x3ff0000000000000ULL); /* log2(2)=1 */
    check_bitexact_u64("fyl2xp1(3, 1)",   do_fyl2xp1(3.0, 1.0),   0x4008000000000000ULL); /* 3*log2(2)=3 */
    check_bitexact_u64("fyl2xp1(1, 3)",   do_fyl2xp1(1.0, 3.0),   0x4000000000000000ULL); /* log2(4)=2 */

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
