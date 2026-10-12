/*
 * test_alloc_decline.c: x87 shapes for the allocation-failure decline path.
 *
 * When the translator cannot get a scratch register it drops everything the
 * request emitted and declines it, and stock Rosetta translates that
 * instruction and the rest of its block instead.  run_tests.sh runs this
 * test (and a few others) with X87_FAIL_ALLOC_EVERY=N, which fails every Nth
 * allocation, at several N, so declines land at the run boundaries, in the
 * middle of IR lowering, inside a bridged region, inside a transcendental
 * and between a FUCOMI and the FCMOVU that reads its flags.  Whatever gets
 * declined, the results and the x87 stack must be exactly what they are
 * without it.  Each shape runs several times so the translated code is also
 * checked on reuse.
 *
 * Inputs are small dyadic values, so every exact check holds bit for bit in
 * binary64 whichever path computed it.
 *
 * Build: clang -arch x86_64 -O2 -o test_alloc_decline test_alloc_decline.c
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;
static int iteration = 0;

static uint64_t as_u64(double d) {
    uint64_t u;
    memcpy(&u, &d, 8);
    return u;
}

static void report(const char* name, int ok, const char* detail) {
    if (!ok) {
        printf("FAIL  %-48s  iteration %d  %s\n", name, iteration, detail);
        failures++;
    } else if (iteration == 0) {
        printf("PASS  %s\n", name);
    }
}

static void check(const char* name, double got, double expected) {
    char detail[96];
    snprintf(detail, sizeof(detail), "got=%.17g expected=%.17g", got, expected);
    report(name, as_u64(got) == as_u64(expected), detail);
}

static void check_close(const char* name, double got, double expected) {
    char detail[96];
    snprintf(detail, sizeof(detail), "got=%.17g expected=%.17g", got, expected);
    report(name, fabs(got - expected) <= 1e-14 * fabs(expected), detail);
}

static void check_u64(const char* name, uint64_t got, uint64_t expected) {
    char detail[96];
    snprintf(detail, sizeof(detail), "got=0x%llx expected=0x%llx", (unsigned long long)got,
             (unsigned long long)expected);
    report(name, got == expected, detail);
}

/* TOP and the tag word after a shape: a declined instruction that lost or
 * repeated a push or pop shows up here even when the values happen to
 * survive. */
static void check_stack_empty(const char* name) {
    uint16_t sw;
    uint8_t env[28];
    __asm__ volatile("fnstsw %0" : "=m"(sw));
    __asm__ volatile("fnstenv %0\n\tfldenv %0" : "+m"(env));
    uint16_t tag;
    memcpy(&tag, env + 8, 2);
    char label[80];
    snprintf(label, sizeof(label), "%s: stack empty after", name);
    check_u64(label, ((uint64_t)((sw >> 11) & 7) << 16) | tag, 0xFFFF);
}

/* One long run: deferred FXCH, a non-popping op on ST(1), pops in the middle
 * and a store that leaves a value behind for the run's tail. */
static void long_run(const double* a, double* out) {
    __asm__ volatile(
        "fldl   (%1)\n\t"
        "fldl   8(%1)\n\t"
        "fldl   16(%1)\n\t"
        "fxch   %%st(2)\n\t"
        "fsubl  24(%1)\n\t"
        "fmul   %%st(1), %%st\n\t"
        "fxch   %%st(1)\n\t"
        "faddl  32(%1)\n\t"
        "fmulp\n\t"
        "fadd   %%st(1), %%st\n\t"
        "fstpl  (%0)\n\t"
        "fdivl  40(%1)\n\t"
        "fstpl  8(%0)\n\t"
        :
        : "r"(out), "r"(a)
        : "st", "st(1)", "st(2)", "memory");
}

/* Eight products live at once: the shape the pressure gate splits. */
static double product_tree(const double* a, const double* b) {
    double out;
    __asm__ volatile(
        "fldl   (%1)\n\t"
        "fmull  56(%2)\n\t"
        "fldl   8(%1)\n\t"
        "fmull  40(%2)\n\t"
        "fldl   16(%1)\n\t"
        "fmull  24(%2)\n\t"
        "fldl   24(%1)\n\t"
        "fmull  8(%2)\n\t"
        "fldl   32(%1)\n\t"
        "fmull  48(%2)\n\t"
        "fldl   40(%1)\n\t"
        "fmull  32(%2)\n\t"
        "fldl   48(%1)\n\t"
        "fmull  16(%2)\n\t"
        "fldl   56(%1)\n\t"
        "fmull  (%2)\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "faddp\n\t"
        "fstpl  %0\n\t"
        : "=m"(out)
        : "r"(a), "r"(b)
        : "st", "st(1)", "st(2)", "st(3)", "st(4)", "st(5)", "st(6)", "st(7)");
    return out;
}

/* Two x87 segments joined by mov/lea, which run bridging lowers as one IR
 * run.  The integer result checks the bridge instructions themselves. */
static double bridged(const double* a, uint32_t* edx_out) {
    double out;
    __asm__ volatile(
        "fldl   (%2)\n\t"
        "fmull  8(%2)\n\t"
        "movl   $0x1234, %%ecx\n\t"
        "leal   8(%%ecx,%%ecx,2), %%edx\n\t"
        "fldl   16(%2)\n\t"
        "fmull  24(%2)\n\t"
        "faddp\n\t"
        "fldl   32(%2)\n\t"
        "fmulp\n\t"
        "fstpl  %0\n\t"
        "movl   %%edx, %1\n\t"
        : "=m"(out), "=m"(*edx_out)
        : "r"(a)
        : "ecx", "edx", "st", "st(1)", "memory");
    return out;
}

/* Integer stores and rounding inside one run. */
static void integer_stores(const double* a, int32_t* i32, int64_t* i64, int32_t* t32) {
    __asm__ volatile(
        "fldl    (%3)\n\t"
        "fmull   8(%3)\n\t"
        "fld     %%st(0)\n\t"
        "frndint\n\t"
        "fistpl  (%0)\n\t"
        "fldl    16(%3)\n\t"
        "faddp\n\t"
        "fld     %%st(0)\n\t"
        "fisttpl (%2)\n\t"
        "fistpll (%1)\n\t"
        :
        : "r"(i32), "r"(i64), "r"(t32), "r"(a)
        : "st", "st(1)", "memory");
}

/* FCOMI and FCOM+FNSTSW in the middle of a run. */
static void compares(const double* a, uint8_t* below, uint16_t* sw) {
    __asm__ volatile(
        "fldl    (%2)\n\t"
        "fldl    8(%2)\n\t"
        "fucomip %%st(1), %%st\n\t"
        "setb    %0\n\t"
        "fstp    %%st(0)\n\t"
        "fldl    16(%2)\n\t"
        "fmull   24(%2)\n\t"
        "fcompl  8(%2)\n\t"
        "fnstsw  %%ax\n\t"
        "movw    %%ax, %1\n\t"
        : "=m"(*below), "=m"(*sw)
        : "r"(a)
        : "eax", "st", "st(1)", "cc", "memory");
}

/* FUCOMI sets PF and FCMOVU reads it, from NZCV bit 26 in both stock's and
 * the sidecar's translation, so the result holds whichever of them a
 * declined request leaves each instruction to. */
static double fcmovu_after_fucomi(double a, double b, double src) {
    double out;
    __asm__ volatile(
        "fldl    %3\n\t"
        "fldl    %2\n\t"
        "fldl    %1\n\t"
        "fucomi  %%st(1), %%st\n\t"
        "fcmovu  %%st(2), %%st\n\t"
        "fstpl   %0\n\t"
        "fstp    %%st(0)\n\t"
        "fstp    %%st(0)\n\t"
        : "=m"(out)
        : "m"(a), "m"(b), "m"(src)
        : "st", "st(1)", "st(2)", "cc");
    return out;
}

/* Exact unary ops and FSCALE, then a transcendental, in one run. */
static void unary_and_sin(const double* a, double* out) {
    __asm__ volatile(
        "fldl   (%1)\n\t"
        "fsqrt\n\t"
        "fldl   8(%1)\n\t"
        "fxch   %%st(1)\n\t"
        "fscale\n\t"
        "fchs\n\t"
        "fabs\n\t"
        "faddp\n\t"
        "fstpl  (%0)\n\t"
        "fldl   16(%1)\n\t"
        "fsin\n\t"
        "fmull  24(%1)\n\t"
        "fstpl  8(%0)\n\t"
        :
        : "r"(out), "r"(a)
        : "st", "st(1)", "memory");
}

int main(void) {
    for (iteration = 0; iteration < 3; iteration++) {
        {
            const double a[6] = {1.5, -2.25, 3.0, 0.5, -4.75, 2.0};
            double out[2];
            long_run(a, out);
            const double t = (a[0] - a[3]) * a[1];
            const double u = (a[1] + a[4]) * t;
            check("long run: first store", out[0], u + a[2]);
            check("long run: second store", out[1], a[2] / a[5]);
            check_stack_empty("long run");
        }
        {
            const double a[8] = {1.5, -2.25, 3.0, 0.5, -4.75, 6.125, -0.375, 8.0};
            const double b[8] = {2.0, 0.25, -1.5, 3.75, 5.0, -0.125, 7.5, -2.5};
            const double p[8] = {a[0] * b[7], a[1] * b[5], a[2] * b[3], a[3] * b[1],
                                 a[4] * b[6], a[5] * b[4], a[6] * b[2], a[7] * b[0]};
            double acc = p[7];
            for (int i = 6; i >= 0; i--) {
                acc = p[i] + acc;
            }
            check("product tree (8 live values)", product_tree(a, b), acc);
            check_stack_empty("product tree");
        }
        {
            const double a[5] = {1.5, 2.5, -3.25, 4.0, 0.75};
            uint32_t edx = 0;
            const double got = bridged(a, &edx);
            check("bridged region: fp result", got, (a[0] * a[1] + a[2] * a[3]) * a[4]);
            check_u64("bridged region: lea result", edx, 8 + 3 * 0x1234);
            check_stack_empty("bridged region");
        }
        {
            const double a[3] = {2.5, 3.0, -1.25};
            int32_t i32 = 0;
            int64_t i64 = 0;
            int32_t t32 = 0;
            integer_stores(a, &i32, &i64, &t32);
            check_u64("integer stores: frndint + fistpl", (uint64_t)(int64_t)i32, 8);
            check_u64("integer stores: fistpll", (uint64_t)i64, 6);
            check_u64("integer stores: fisttpl", (uint64_t)(int64_t)t32, 6);
            check_stack_empty("integer stores");
        }
        {
            const double a[4] = {2.0, 1.0, 0.25, 2.0};
            uint8_t below = 0;
            uint16_t sw = 0;
            compares(a, &below, &sw);
            check_u64("compares: fucomip below", below, 1);
            /* C3, C2, C0 and TOP; C1 is left out. */
            check_u64("compares: fcompl status", sw & 0x7D00U, 0x0100);
            check_stack_empty("compares");
        }
        {
            check("fcmovu after fucomi: unordered moves", fcmovu_after_fucomi(NAN, 1.0, 99.0),
                  99.0);
            check("fcmovu after fucomi: ordered keeps", fcmovu_after_fucomi(5.0, 1.0, 99.0), 5.0);
            check_stack_empty("fcmovu after fucomi");
        }
        {
            const double a[4] = {2.25, 3.0, 0.5, 1.5};
            double out[2];
            unary_and_sin(a, out);
            check("unary + fscale", out[0], 12.0 + 3.0);
            check_close("fsin in a run", out[1], sin(a[2]) * a[3]);
            check_stack_empty("unary + fsin");
        }
    }

    printf("%d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
