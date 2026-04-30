/*
 * test_fsin.c — FSIN routed through sidecar IPC + libm.
 *
 * The sidecar's receive loop dispatches msgh_id=0x10000002 to
 * runTranscendental, which calls std::sin via <cmath>.  Since the
 * test process and the sidecar both link the same libm, results are
 * **bit-exact** — compare via memcpy + uint64_t (==' is wrong: it
 * collapses ±0 and misses NaN payloads).
 *
 * Native Rosetta's stock fsin uses x87 80-bit precision, which differs
 * from libm double in low bits — that's expected. This test exercises
 * the JIT path only.
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

static double do_fsin(double v) {
    double r;
    __asm__ volatile(
        "fldl  %1\n\t"
        "fsin\n\t"
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(v)
        : "st");
    return r;
}

static double do_fadd_then_sin(double a, double b) {
    /* a + b, then sin(a+b) — exercises the cache (faddp consumes stack)
       BEFORE the sidecar IPC fires.  Confirms cache state survives the
       BLR roundtrip. */
    double r;
    __asm__ volatile(
        "fldl  %1\n\t"
        "fldl  %2\n\t"
        "faddp %%st, %%st(1)\n\t"
        "fsin\n\t"
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(a), "m"(b)
        : "st");
    return r;
}

int main(void) {
    /* Minimal shape — fsin after just fld.  Inputs chosen so x87's
       80-bit fsin and libm's f64 sin produce bit-exact same f64 — so
       this test passes both natively (under stock x87) and under the
       sidecar IPC path (libm).  M_PI is intentionally avoided: f64(π)
       is below the true π by ~1.2e-16, and x87's 80-bit sin produces
       a slightly different low-bit f64 result there. */
    check_bitexact("fsin(0.0)",         do_fsin(0.0),         sin(0.0));
    check_bitexact("fsin(-0.0)",        do_fsin(-0.0),        sin(-0.0));
    check_bitexact("fsin(1.0)",         do_fsin(1.0),         sin(1.0));
    check_bitexact("fsin(0.5)",         do_fsin(0.5),         sin(0.5));
    check_bitexact("fsin(M_PI/2)",      do_fsin(M_PI / 2.0),  sin(M_PI / 2.0));
    check_bitexact("fsin(-1.0)",        do_fsin(-1.0),        sin(-1.0));

    /* Boundary shape — handled prefix (faddp) writes our cache, then
       fsin's IPC fires with cache state in registers. */
    check_bitexact("fsin(0.5+0.5)",     do_fadd_then_sin(0.5, 0.5),  sin(1.0));
    check_bitexact("fsin(0.0+0.0)",     do_fadd_then_sin(0.0, 0.0),  sin(0.0));
    check_bitexact("fsin(0.3+0.4)",     do_fadd_then_sin(0.3, 0.4),  sin(0.3 + 0.4));

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
