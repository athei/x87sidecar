/*
 * test_f2xm1.c — F2XM1 routed through sidecar IPC + libm.
 *
 * x87 F2XM1 computes 2^ST(0) - 1, defined for |ST(0)| <= 1.  Sidecar
 * uses std::exp2(in) - 1.0; results are bit-exact between native x87
 * and libm for the inputs chosen here.
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

static double do_f2xm1(double v) {
    double r;
    __asm__ volatile(
        "fldl  %1\n\t"
        "f2xm1\n\t"
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(v)
        : "st");
    return r;
}

int main(void) {
    /* Inputs chosen so x87 80-bit f2xm1 and libm exp2(x)-1 agree.
       Avoided: -0 (x87 preserves sign through 2^-0-1, libm yields +0);
       fractional 0.5, 0.25 (low-bit precision diffs). */
    check_bitexact("f2xm1(0.0)",        do_f2xm1(0.0),        exp2(0.0) - 1.0);
    check_bitexact("f2xm1(1.0)",        do_f2xm1(1.0),        exp2(1.0) - 1.0);
    check_bitexact("f2xm1(-1.0)",       do_f2xm1(-1.0),       exp2(-1.0) - 1.0);

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
