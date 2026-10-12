/*
 * test_ir_fpatan_rc.c: fpatan in a run that caches the rounding control.
 *
 * A run with two or more rounding integer stores (and no fcom/ftst) keeps
 * the RC field in a pinned GPR for the whole run, which leaves three of the
 * eight scratch GPRs free at a fresh run start.  The inline fpatan needs
 * four at once: the constants pointer, the result sign, the saved NZCV and
 * one more for the 1.0 constant or the sign test of x.  The IR GPR model
 * has to count all four, or the gate passes the run and the lowering runs
 * out of GPRs.
 *
 * Results are scaled before the integer stores so a few ULP of difference
 * in atan2 cannot change them.  The live GPR pool depends on stock's
 * register state, so tests/data/ir_fpatan_rc.ir holds these blocks for
 * ir_pressure_replay, which run_tests.sh replays on the full 8-GPR pool.
 *
 * Build: clang -arch x86_64 -O0 -o test_ir_fpatan_rc test_ir_fpatan_rc.c -lm
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static int failures = 0;

static void check_i32(const char* name, int32_t got, int32_t expected) {
    if (got != expected) {
        printf("FAIL  %-52s  got=%d  expected=%d\n", name, got, expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

#define CW_NEAREST 0x037F
#define CW_DOWN 0x077F

static void set_cw(uint16_t cw) {
    __asm__ volatile("fldcw %0" : : "m"(cw));
}

/* out[0] = out[1] = rint(atan2(y, x) * scale), via fistl then fistpl. */
static void fpatan_fist_fistp(const double* v, int32_t* out) {
    __asm__ volatile(
        "fldl   (%1)\n\t"  /* y */
        "fldl   8(%1)\n\t" /* x */
        "fpatan\n\t"
        "fmull  16(%1)\n\t" /* scale */
        "fistl  (%0)\n\t"
        "fistpl 4(%0)\n"
        :
        : "r"(out), "r"(v)
        : "memory", "st", "st(1)");
}

/* The same with the stores before and after the fpatan. */
static void fist_fpatan_fistp(const double* v, int32_t* out) {
    __asm__ volatile(
        "fldl   16(%1)\n\t" /* scale */
        "fistl  (%0)\n\t"
        "fldl   (%1)\n\t"  /* y */
        "fldl   8(%1)\n\t" /* x */
        "fpatan\n\t"
        "fmulp\n\t"
        "fistpl 4(%0)\n"
        :
        : "r"(out), "r"(v)
        : "memory", "st", "st(1)", "st(2)");
}

int main(void) {
    char name[96];
    static const double cases[][2] = {
        {1.0, 1.0},
        {1.0, -1.0},
        {-3.0, 0.5},
        {0.0, -2.0},
    };
    const double scale = 1000.0;

    for (int mode = 0; mode < 2; mode++) {
        const uint16_t cw = mode == 0 ? CW_NEAREST : CW_DOWN;
        const char* rc = mode == 0 ? "RN" : "RD";
        set_cw(cw);
        for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            const double v[3] = {cases[i][0], cases[i][1], scale};
            const double r = atan2(cases[i][0], cases[i][1]) * scale;
            const int32_t expected = (int32_t)(mode == 0 ? nearbyint(r) : floor(r));

            int32_t out[2] = {0, 0};
            fpatan_fist_fistp(v, out);
            snprintf(name, sizeof(name), "%s fpatan;fist;fistp  atan2(%g, %g)", rc, v[0], v[1]);
            check_i32(name, out[0], expected);
            snprintf(name, sizeof(name), "%s fpatan;fist;fistp  atan2(%g, %g) popped", rc, v[0],
                     v[1]);
            check_i32(name, out[1], expected);

            int32_t out2[2] = {0, 0};
            fist_fpatan_fistp(v, out2);
            snprintf(name, sizeof(name), "%s fist;fpatan;fistp  scale", rc);
            check_i32(name, out2[0], (int32_t)scale);
            snprintf(name, sizeof(name), "%s fist;fpatan;fistp  atan2(%g, %g)", rc, v[0], v[1]);
            check_i32(name, out2[1], expected);
        }
    }
    set_cw(CW_NEAREST);

    printf("%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
