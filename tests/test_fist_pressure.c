/*
 * test_fist_pressure.c: integer stores while every pool FPR is live.
 *
 * Each case loads eight values, so the run holds eight FPRs when its first
 * integer store is emitted.  The store's indefinite guard needs one more
 * scratch FPR while its input is still held, so the IR pressure model has
 * to count that register: if it does not, an eight-register pool passes the
 * gate and the lowering runs out of FPRs inside the guard.
 *
 * The values include NaN and out-of-range inputs so the guard's indefinite
 * paths run, and in-range ones that round under round-to-nearest.  They are
 * loaded with fldl rather than fildll so the stores stay real conversions.
 *
 * tests/data/fist_pressure.ir is these blocks captured for
 * ir_pressure_replay, which run_tests.sh replays on an 8-FPR pool; the live
 * pool depends on stock's FPR seeding.
 *
 * Build: clang -arch x86_64 -O0 -o test_fist_pressure test_fist_pressure.c
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static int failures = 0;

#define INDEF16 ((int16_t)0x8000)
#define INDEF32 ((int32_t)0x80000000)
#define INDEF64 ((int64_t)0x8000000000000000ULL)

static void check_i16(const char* name, int16_t got, int16_t expected) {
    if (got != expected) {
        printf("FAIL  %-52s  got=0x%04x  expected=0x%04x\n", name, (uint16_t)got,
               (uint16_t)expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

static void check_i32(const char* name, int32_t got, int32_t expected) {
    if (got != expected) {
        printf("FAIL  %-52s  got=0x%08x  expected=0x%08x\n", name, (uint32_t)got,
               (uint32_t)expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

static void check_i64(const char* name, int64_t got, int64_t expected) {
    if (got != expected) {
        printf("FAIL  %-52s  got=0x%016llx  expected=0x%016llx\n", name, (unsigned long long)got,
               (unsigned long long)expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

#define CW_NEAREST 0x037F

static void set_cw(uint16_t cw) {
    __asm__ volatile("fldcw %0" : : "m"(cw));
}

/* Eight loads, then eight popping m64 stores.  out[i] receives in[i]. */
static void fistpll_x8(const double* in, int64_t* out) {
    __asm__ volatile(
        "fldl    (%1)\n\t"
        "fldl    8(%1)\n\t"
        "fldl    16(%1)\n\t"
        "fldl    24(%1)\n\t"
        "fldl    32(%1)\n\t"
        "fldl    40(%1)\n\t"
        "fldl    48(%1)\n\t"
        "fldl    56(%1)\n\t"
        "fistpll 56(%0)\n\t"
        "fistpll 48(%0)\n\t"
        "fistpll 40(%0)\n\t"
        "fistpll 32(%0)\n\t"
        "fistpll 24(%0)\n\t"
        "fistpll 16(%0)\n\t"
        "fistpll 8(%0)\n\t"
        "fistpll (%0)\n"
        :
        : "r"(out), "r"(in)
        : "memory");
}

/* Eight loads, then eight popping m32 stores.  out[i] receives in[i]. */
static void fistpl_x8(const double* in, int32_t* out) {
    __asm__ volatile(
        "fldl   (%1)\n\t"
        "fldl   8(%1)\n\t"
        "fldl   16(%1)\n\t"
        "fldl   24(%1)\n\t"
        "fldl   32(%1)\n\t"
        "fldl   40(%1)\n\t"
        "fldl   48(%1)\n\t"
        "fldl   56(%1)\n\t"
        "fistpl 28(%0)\n\t"
        "fistpl 24(%0)\n\t"
        "fistpl 20(%0)\n\t"
        "fistpl 16(%0)\n\t"
        "fistpl 12(%0)\n\t"
        "fistpl 8(%0)\n\t"
        "fistpl 4(%0)\n\t"
        "fistpl (%0)\n"
        :
        : "r"(out), "r"(in)
        : "memory");
}

/* Eight loads, then eight popping m16 stores.  out[i] receives in[i]. */
static void fistps_x8(const double* in, int16_t* out) {
    __asm__ volatile(
        "fldl   (%1)\n\t"
        "fldl   8(%1)\n\t"
        "fldl   16(%1)\n\t"
        "fldl   24(%1)\n\t"
        "fldl   32(%1)\n\t"
        "fldl   40(%1)\n\t"
        "fldl   48(%1)\n\t"
        "fldl   56(%1)\n\t"
        "fistps 14(%0)\n\t"
        "fistps 12(%0)\n\t"
        "fistps 10(%0)\n\t"
        "fistps 8(%0)\n\t"
        "fistps 6(%0)\n\t"
        "fistps 4(%0)\n\t"
        "fistps 2(%0)\n\t"
        "fistps (%0)\n"
        :
        : "r"(out), "r"(in)
        : "memory");
}

/* Eight loads, then eight truncating m64 stores.  out[i] receives in[i]. */
static void fisttpll_x8(const double* in, int64_t* out) {
    __asm__ volatile(
        "fldl     (%1)\n\t"
        "fldl     8(%1)\n\t"
        "fldl     16(%1)\n\t"
        "fldl     24(%1)\n\t"
        "fldl     32(%1)\n\t"
        "fldl     40(%1)\n\t"
        "fldl     48(%1)\n\t"
        "fldl     56(%1)\n\t"
        "fisttpll 56(%0)\n\t"
        "fisttpll 48(%0)\n\t"
        "fisttpll 40(%0)\n\t"
        "fisttpll 32(%0)\n\t"
        "fisttpll 24(%0)\n\t"
        "fisttpll 16(%0)\n\t"
        "fisttpll 8(%0)\n\t"
        "fisttpll (%0)\n"
        :
        : "r"(out), "r"(in)
        : "memory");
}

/* Eight loads, a non-popping m32 store of ST(0) to out[8] with all eight
 * still live, then eight popping m32 stores.  out[i] receives in[i]. */
static void fistl_x8(const double* in, int32_t* out) {
    __asm__ volatile(
        "fldl   (%1)\n\t"
        "fldl   8(%1)\n\t"
        "fldl   16(%1)\n\t"
        "fldl   24(%1)\n\t"
        "fldl   32(%1)\n\t"
        "fldl   40(%1)\n\t"
        "fldl   48(%1)\n\t"
        "fldl   56(%1)\n\t"
        "fistl  32(%0)\n\t"
        "fistpl 28(%0)\n\t"
        "fistpl 24(%0)\n\t"
        "fistpl 20(%0)\n\t"
        "fistpl 16(%0)\n\t"
        "fistpl 12(%0)\n\t"
        "fistpl 8(%0)\n\t"
        "fistpl 4(%0)\n\t"
        "fistpl (%0)\n"
        :
        : "r"(out), "r"(in)
        : "memory");
}

int main(void) {
    const double qnan = NAN;
    char name[96];

    set_cw(CW_NEAREST);

    /* ── m64 ─────────────────────────────────────────────────────────────── */
    {
        const double in[8] = {
            qnan,                    /* NaN → indefinite */
            1e30,                    /* + out of range → indefinite */
            -1e30,                   /* - out of range → indefinite */
            9223372036854775808.0,   /* 2^63 → indefinite */
            -9223372036854775808.0,  /* -2^63, in range */
            9223372036854774784.0,   /* 2^63 - 1024, in range */
            12.5,                    /* RN ties to even → 12 */
            -7.5,                    /* RN ties to even → -8 */
        };
        const int64_t expected[8] = {
            INDEF64, INDEF64, INDEF64, INDEF64, INT64_MIN, 9223372036854774784LL, 12, -8,
        };
        int64_t out[8] = {0};
        fistpll_x8(in, out);
        for (int i = 0; i < 8; i++) {
            snprintf(name, sizeof(name), "fistpll x8  in[%d]=%g", i, in[i]);
            check_i64(name, out[i], expected[i]);
        }
    }

    /* ── m32 ─────────────────────────────────────────────────────────────── */
    const double in32[8] = {
        qnan,             /* NaN → indefinite */
        1099511627776.0,  /* 2^40 → indefinite, not INT_MAX */
        -1099511627776.0, /* -2^40 → indefinite */
        2147483647.5,     /* RN rounds to 2^31 → indefinite */
        2147483647.0,     /* INT32_MAX, in range */
        -2147483648.0,    /* INT32_MIN, in range */
        3.5,              /* RN ties to even → 4 */
        -2.5,             /* RN ties to even → -2 */
    };
    const int32_t expected32[8] = {
        INDEF32, INDEF32, INDEF32, INDEF32, 2147483647, INT32_MIN, 4, -2,
    };
    {
        int32_t out[8] = {0};
        fistpl_x8(in32, out);
        for (int i = 0; i < 8; i++) {
            snprintf(name, sizeof(name), "fistpl x8  in[%d]=%g", i, in32[i]);
            check_i32(name, out[i], expected32[i]);
        }
    }

    /* ── Non-popping store with eight values live ────────────────────────── */
    {
        int32_t out[9] = {0};
        fistl_x8(in32, out);
        check_i32("fistl x8  ST(0)=-2.5 before the pops", out[8], -2);
        for (int i = 0; i < 8; i++) {
            snprintf(name, sizeof(name), "fistl x8  in[%d]=%g", i, in32[i]);
            check_i32(name, out[i], expected32[i]);
        }
    }
    {
        /* Same shape with the non-popping store hitting NaN. */
        const double in[8] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, qnan};
        const int32_t expected[8] = {1, 2, 3, 4, 5, 6, 7, INDEF32};
        int32_t out[9] = {0};
        fistl_x8(in, out);
        check_i32("fistl x8  ST(0)=NaN before the pops", out[8], INDEF32);
        for (int i = 0; i < 8; i++) {
            snprintf(name, sizeof(name), "fistl x8 nan  in[%d]=%g", i, in[i]);
            check_i32(name, out[i], expected[i]);
        }
    }

    /* ── m16 ─────────────────────────────────────────────────────────────── */
    {
        const double in[8] = {
            qnan,     /* NaN → indefinite */
            32768.0,  /* 2^15 → indefinite */
            32767.5,  /* RN rounds to 2^15 → indefinite */
            -32769.0, /* below INT16_MIN → indefinite */
            32767.0,  /* INT16_MAX, in range */
            -32768.0, /* INT16_MIN, in range */
            12.5,     /* RN ties to even → 12 */
            -2.5,     /* RN ties to even → -2 */
        };
        const int16_t expected[8] = {
            INDEF16, INDEF16, INDEF16, INDEF16, 32767, INT16_MIN, 12, -2,
        };
        int16_t out[8] = {0};
        fistps_x8(in, out);
        for (int i = 0; i < 8; i++) {
            snprintf(name, sizeof(name), "fistps x8  in[%d]=%g", i, in[i]);
            check_i16(name, out[i], expected[i]);
        }
    }

    /* ── FISTTP m64 (truncating, ignores RC) ─────────────────────────────── */
    {
        const double in[8] = {
            qnan,                   /* NaN → indefinite */
            1e30,                   /* + out of range → indefinite */
            9223372036854775808.0,  /* 2^63 → indefinite */
            -9223372036854775808.0, /* -2^63, in range */
            9223372036854774784.0,  /* 2^63 - 1024, in range */
            12.9,                   /* truncates to 12 */
            -7.9,                   /* truncates to -7 */
            0.5,                    /* truncates to 0 */
        };
        const int64_t expected[8] = {
            INDEF64, INDEF64, INDEF64, INT64_MIN, 9223372036854774784LL, 12, -7, 0,
        };
        int64_t out[8] = {0};
        fisttpll_x8(in, out);
        for (int i = 0; i < 8; i++) {
            snprintf(name, sizeof(name), "fisttpll x8  in[%d]=%g", i, in[i]);
            check_i64(name, out[i], expected[i]);
        }
    }

    printf("%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
