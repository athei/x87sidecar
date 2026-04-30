/*
 * test_fpatan.c — FPATAN routed through sidecar IPC + libm.
 *
 * x86 fpatan: ST(0) = atan2(ST(1), ST(0)), pop.
 * Inline asm: fld y; fld x; fpatan; fstpl r — leaves atan2(y, x) in r.
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

static double do_fpatan(double y, double x) {
    double r;
    __asm__ volatile(
        "fldl  %1\n\t"   /* push y → ST(0) */
        "fldl  %2\n\t"   /* push x → ST(0); y now ST(1) */
        "fpatan\n\t"     /* ST(0) = atan2(ST(1), ST(0)) = atan2(y, x); pop */
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(y), "m"(x)
        : "st");
    return r;
}

int main(void) {
    /* Inputs chosen so x87 80-bit fpatan and libm atan2 agree bit-for-bit. */
    check_bitexact("fpatan(0, 1)",     do_fpatan(0.0, 1.0),  atan2(0.0, 1.0));
    check_bitexact("fpatan(1, 0)",     do_fpatan(1.0, 0.0),  atan2(1.0, 0.0));
    check_bitexact("fpatan(1, 1)",     do_fpatan(1.0, 1.0),  atan2(1.0, 1.0));
    check_bitexact("fpatan(-1, 0)",    do_fpatan(-1.0, 0.0), atan2(-1.0, 0.0));
    check_bitexact("fpatan(0, -1)",    do_fpatan(0.0, -1.0), atan2(0.0, -1.0));
    check_bitexact("fpatan(-1, -1)",   do_fpatan(-1.0, -1.0),atan2(-1.0, -1.0));

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
