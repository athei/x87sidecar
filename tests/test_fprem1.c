/*
 * test_fprem1.c — FPREM1 routed through sidecar IPC + libm.
 *
 * x86 fprem1: ST(0) := IEEE remainder(ST(0), ST(1)).  Quotient rounds
 * to nearest-even (vs fprem's truncate-toward-zero).  No pop.
 *
 * Same iterative simplification as fprem.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static int check_bitexact(const char* name, double got, double expected) {
    uint64_t g, e;
    memcpy(&g, &got, sizeof(g));
    memcpy(&e, &expected, sizeof(e));
    if (g == e) {
        printf("PASS  %-40s  got=0x%016llx (%.17g)\n", name, (unsigned long long)g, got);
        return 1;
    }
    printf("FAIL  %-40s  got=0x%016llx (%.17g)  expected=0x%016llx (%.17g)\n", name,
           (unsigned long long)g, got, (unsigned long long)e, expected);
    failures++;
    return 0;
}

static int check_bitexact_u64(const char* name, double got, uint64_t expected_bits) {
    double expected;
    memcpy(&expected, &expected_bits, sizeof(expected));
    return check_bitexact(name, got, expected);
}

static double do_fprem1(double x, double y) {
    double r;
    __asm__ volatile(
        "fldl  %1\n\t"
        "fldl  %2\n\t"
        "fprem1\n\t"
        "fstpl %0\n\t"
        "ffree %%st(0)\n\t"
        "fincstp\n\t"
        : "=m"(r)
        : "m"(y), "m"(x)
        : "st");
    return r;
}

/* See test_fprem.c — fprem1 shares the iterative-completion contract
 * and must clear C2 in the status word, or wine's reduction loop
 * spins forever (the WoW world-load freeze). */
static uint16_t do_fprem1_sw_after(double x, double y) {
    uint16_t sw;
    double r;
    __asm__ volatile(
        "fldl  %2\n\t"
        "fldl  %3\n\t"
        "fxam\n\t"   /* x is normal → SW.C2 := 1 */
        "fprem1\n\t" /* must clear SW.C2 */
        "fnstsw %%ax\n\t"
        "movw  %%ax, %0\n\t"
        "fstpl %1\n\t"
        "ffree %%st(0)\n\t"
        "fincstp\n\t"
        : "=m"(sw), "=m"(r)
        : "m"(y), "m"(x)
        : "ax", "st");
    (void)r;
    return sw;
}

static void check_c2_clear(const char* name, uint16_t sw) {
    if ((sw & 0x0400U) == 0U) {
        printf("PASS  %-40s  sw=0x%04x  C2=0\n", name, (unsigned)sw);
    } else {
        printf("FAIL  %-40s  sw=0x%04x  C2=1 (fprem1 must clear C2)\n", name, (unsigned)sw);
        failures++;
    }
}

int main(void) {
    /* IEEE remainder rounds quotient to nearest-even, so:
       remainder(5, 3) = 5 - round(5/3)*3 = 5 - 2*3 = -1 (vs fmod = 2)
       remainder(7, 3) = 7 - round(7/3)*3 = 7 - 2*3 = 1
       remainder(8, 3) = 8 - round(8/3)*3 = 8 - 3*3 = -1 (vs fmod = 2) */
    check_bitexact_u64("fprem1(5, 3)", do_fprem1(5.0, 3.0), 0xbff0000000000000ULL); /* -1 */
    check_bitexact_u64("fprem1(7, 3)", do_fprem1(7.0, 3.0), 0x3ff0000000000000ULL); /* 1 */
    check_bitexact_u64("fprem1(8, 3)", do_fprem1(8.0, 3.0), 0xbff0000000000000ULL); /* -1 */
    check_bitexact_u64("fprem1(1, 1)", do_fprem1(1.0, 1.0), 0x0000000000000000ULL); /* 0 */
    check_bitexact_u64("fprem1(6, 4)", do_fprem1(6.0, 4.0),
                       0xc000000000000000ULL); /* -2 (round-to-even) */

    check_c2_clear("fprem1(5, 3) clears C2", do_fprem1_sw_after(5.0, 3.0));
    check_c2_clear("fprem1(7, 3) clears C2", do_fprem1_sw_after(7.0, 3.0));
    check_c2_clear("fprem1(-5, 3) clears C2", do_fprem1_sw_after(-5.0, 3.0));

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
