/*
 * test_fsin.c — Minimal regression for x87 FSIN forwarded to stock.
 *
 * Build: clang -arch x86_64 -O0 -o test_fsin test_fsin.c -lm
 *
 * Goal of this test (Phase 1 of plan):
 *   Reproduce the boundary failure for an opcode our Translator forwards
 *   to stock. FSIN is unhandled by translate_*; the dispatcher hits its
 *   default case, calls x87_cache_force_release, returns nullopt, the
 *   IPC stub falls through to STASH+stock, and stock's translate_insn
 *   emits BL kRuntimeRoutine_fsin which (in sidecar mode) hits stock's
 *   helper.
 *
 *   We expect this to work on the dylib path (../rosettax87_jit_main)
 *   and to fail on dev. Two shapes:
 *     do_fsin           — minimal: fld + fsin + fstp. Our handled-prefix
 *                         emit is just translate_fld + translate_fst.
 *     do_fadd_then_sin  — fld + fld + faddp + fsin + fstp. Exercises the
 *                         x87 cache (TOP, deferred state) before the
 *                         boundary, more representative of a hot block.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static int check_close(const char *name, double got, double expected, double tol) {
    double diff = got - expected;
    if (diff < 0) diff = -diff;
    if (diff > tol || got != got /* NaN check */) {
        printf("FAIL  %-40s  got=%.17g  expected=%.17g  diff=%.3g\n",
               name, got, expected, diff);
        failures++;
        return 0;
    }
    printf("PASS  %-40s  got=%.17g\n", name, got);
    return 1;
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
    /* a + b, then sin(a+b) — exercises our cache (faddp consumes stack) */
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
    /* Minimal shape — fsin after just fld. */
    check_close("fsin(0.0)",         do_fsin(0.0),         0.0,                    1e-15);
    check_close("fsin(1.0)",         do_fsin(1.0),         sin(1.0),               1e-15);
    check_close("fsin(0.5)",         do_fsin(0.5),         sin(0.5),               1e-15);
    check_close("fsin(M_PI/2)",      do_fsin(M_PI / 2.0),  1.0,                    1e-15);
    check_close("fsin(-1.0)",        do_fsin(-1.0),        sin(-1.0),              1e-15);

    /* Boundary shape — handled prefix (faddp) writes our cache, then surrender. */
    check_close("fsin(0.5+0.5)",     do_fadd_then_sin(0.5, 0.5),  sin(1.0),         1e-15);
    check_close("fsin(0.0+0.0)",     do_fadd_then_sin(0.0, 0.0),  0.0,              1e-15);
    check_close("fsin(0.3+0.4)",     do_fadd_then_sin(0.3, 0.4),  sin(0.7),         1e-15);

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
