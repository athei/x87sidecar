/*
 * test_fsincos.c — FSINCOS routed through sidecar IPC + libm.
 *
 * x86 fsincos: ST(0) := sin(old ST(0)), then push cos(old ST(0)) → new ST(0).
 * After: sin in ST(1), cos in ST(0).
 *
 * Inline asm: fld v; fsincos; fstpl cos; fstpl sin
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

static void do_fsincos(double v, double *out_sin, double *out_cos) {
    __asm__ volatile(
        "fldl  %2\n\t"
        "fsincos\n\t"
        "fstpl %1\n\t"   /* cos at ST(0) → out_cos */
        "fstpl %0\n\t"   /* sin at new ST(0) → out_sin */
        : "=m"(*out_sin), "=m"(*out_cos)
        : "m"(v)
        : "st");
}

static int check_bitexact_u64(const char *name, double got, uint64_t expected_bits) {
    double expected;
    memcpy(&expected, &expected_bits, sizeof(expected));
    return check_bitexact(name, got, expected);
}

int main(void) {
    /* Expected values are pre-computed (as f64 bit patterns) to avoid
       calling sin()/cos() inside the test — under JIT those calls
       would themselves go through translate_fsin/fcos and risk
       confounding the test if the cache is leaving stale state. */
    double s, c;

    do_fsincos(0.0, &s, &c);
    check_bitexact_u64("fsincos(0).sin", s, 0x0000000000000000ULL);
    check_bitexact_u64("fsincos(0).cos", c, 0x3ff0000000000000ULL);

    do_fsincos(1.0, &s, &c);
    check_bitexact_u64("fsincos(1).sin", s, 0x3feaed548f090ceeULL);
    check_bitexact_u64("fsincos(1).cos", c, 0x3fe14a280fb5068cULL);

    do_fsincos(-1.0, &s, &c);
    check_bitexact_u64("fsincos(-1).sin", s, 0xbfeaed548f090ceeULL);
    check_bitexact_u64("fsincos(-1).cos", c, 0x3fe14a280fb5068cULL);

    do_fsincos(0.5, &s, &c);
    check_bitexact_u64("fsincos(0.5).sin", s, 0x3fdeaee8744b05f0ULL);
    check_bitexact_u64("fsincos(0.5).cos", c, 0x3fec1528065b7d50ULL);

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
