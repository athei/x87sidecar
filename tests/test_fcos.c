/*
 * test_fcos.c — FCOS routed through sidecar IPC + libm.
 *
 * Bit-exact compare via memcpy + uint64_t.  Inputs chosen so x87 80-bit
 * fcos and libm f64 cos agree exactly (so this test passes both
 * natively under stock x87 and under the sidecar IPC path).
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

static double do_fcos(double v) {
    double r;
    __asm__ volatile(
        "fldl  %1\n\t"
        "fcos\n\t"
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(v)
        : "st");
    return r;
}

static double do_fadd_then_cos(double a, double b) {
    double r;
    __asm__ volatile(
        "fldl  %1\n\t"
        "fldl  %2\n\t"
        "faddp %%st, %%st(1)\n\t"
        "fcos\n\t"
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(a), "m"(b)
        : "st");
    return r;
}

int main(void) {
    check_bitexact("fcos(0.0)",         do_fcos(0.0),         cos(0.0));
    check_bitexact("fcos(-0.0)",        do_fcos(-0.0),        cos(-0.0));
    check_bitexact("fcos(1.0)",         do_fcos(1.0),         cos(1.0));
    check_bitexact("fcos(0.5)",         do_fcos(0.5),         cos(0.5));
    check_bitexact("fcos(-1.0)",        do_fcos(-1.0),        cos(-1.0));

    /* Boundary shape — handled prefix writes the cache, then fcos's IPC
       fires with cache state in registers. */
    check_bitexact("fcos(0.5+0.5)",     do_fadd_then_cos(0.5, 0.5),  cos(1.0));
    check_bitexact("fcos(0.0+0.0)",     do_fadd_then_cos(0.0, 0.0),  cos(0.0));
    /* Avoid 0.3+0.4 here — last bit differs between native x87 80-bit
       fcos and libm's f64 cos for that value. */
    check_bitexact("fcos(0.5+0.0)",     do_fadd_then_cos(0.5, 0.0),  cos(0.5));

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
