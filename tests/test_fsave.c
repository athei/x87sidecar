/*
 * test_fsave.c — Tests for FSAVE m108byte (DD /6).
 *
 * fsave stores the current FPU state to a 108-byte buffer (env header +
 * 8 ST slots in x86 f80 format) and re-initializes the FPU.  Functional
 * equivalent: fstenv + finit.
 *
 * Strategy:
 *   1. Push 8 known doubles, fsave to buf.
 *   2. Decode each ST slot from the buffer using `long double` and
 *      compare bit-exact against the originals.
 *   3. Verify post-fsave state via fnstenv: CW=0x037F, SW=0 (TOP=0,
 *      no flags), TW=0xFFFF (all empty).
 *   4. Round-trip: fsave then frstor (now both ours) and verify ST
 *      values come back bit-exact.
 *
 * Lossy paths (inherited from emit_f64_to_f80):
 *   - f64 denormals flush to ±0 in the f80 form.  Don't include
 *     MIN_NORMAL or smaller — 2.5, ±0, ±Inf, NaN, 1.0 are safe.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void fninit(void)             { __asm__ volatile (".byte 0xDB, 0xE3"); }
static void fnsave108(uint8_t *buf)  { __asm__ volatile ("fnsave %0" : "=m"(*buf) :: "memory"); }
static void frstor108(const uint8_t *buf) { __asm__ volatile ("frstor %0" : : "m"(*buf) : "memory"); }
static void fnstenv28(uint8_t *env)  { __asm__ volatile ("fnstenv %0" : "=m"(*env) :: "memory"); }

static void fld_double(const double *d) { __asm__ volatile ("fldl %0" : : "m"(*d)); }
static void fstp_double(double *d)      { __asm__ volatile ("fstpl %0" : "=m"(*d)); }

static uint64_t bits_of(double d) {
    uint64_t u;
    memcpy(&u, &d, sizeof(u));
    return u;
}

static double from_bits(uint64_t u) {
    double d;
    memcpy(&d, &u, sizeof(d));
    return d;
}

/* Decode a 10-byte f80 slot to double via the host long double.
 * On x86_64 SysV, long double is the f80 extended-precision format. */
static double decode_f80(const uint8_t *in10) {
    long double ld;
    memset(&ld, 0, sizeof(ld));
    memcpy(&ld, in10, 10);
    return (double)ld;
}

static void check_bits(const char *name, double got, double expected) {
    uint64_t g = bits_of(got), e = bits_of(expected);
    if (g == e) {
        printf("PASS  %s  bits=0x%016llx\n", name, (unsigned long long)g);
    } else {
        printf("FAIL  %s  got=0x%016llx expected=0x%016llx\n",
               name, (unsigned long long)g, (unsigned long long)e);
        failures++;
    }
}

static void check_eq16(const char *name, uint16_t got, uint16_t expected) {
    if (got == expected) {
        printf("PASS  %s  (0x%04x)\n", name, got);
    } else {
        printf("FAIL  %s  got=0x%04x expected=0x%04x\n", name, got, expected);
        failures++;
    }
}

static uint16_t get16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }

/* ── Test 1: fsave writes correct f80 ST slots, decoded via long double. ── */
static void test_st_save(void) {
    /* Distinct values that exercise sign / zero / inf / NaN.  No denormals
     * (the f64->f80 path flushes them to ±0). */
    const double vals[8] = {
        1.0,
        2.5,
        from_bits(0x8000000000000000ULL),  /* -0.0 */
        from_bits(0x7FF0000000000000ULL),  /* +Inf */
        from_bits(0x7FF8000000000001ULL),  /* qNaN with payload */
        3.141592653589793,
        from_bits(0x7FEFFFFFFFFFFFFFULL),  /* MAX double */
        -42.5,
    };

    fninit();
    /* Push: ST(0)=vals[7], ST(1)=vals[6], ..., ST(7)=vals[0]. */
    for (int i = 0; i < 8; ++i) fld_double(&vals[i]);

    uint8_t buf[108] __attribute__((aligned(16))) = {0};
    fnsave108(buf);

    /* buf[0x1C + i*10] holds ST(i)'s f80.  ST(0)=vals[7], ST(1)=vals[6], ... */
    char name[64];
    for (int i = 0; i < 8; ++i) {
        double got = decode_f80(buf + 0x1C + i * 10);
        snprintf(name, sizeof(name), "fsave: ST(%d) = vals[%d]", i, 7 - i);
        check_bits(name, got, vals[7 - i]);
    }
}

/* ── Test 2: post-fsave state is re-initialized. ───────────────────────── */
static void test_post_fsave_state(void) {
    fninit();
    static const double v[3] = {1.0, 2.0, 3.0};
    for (int i = 0; i < 3; ++i) fld_double(&v[i]);

    uint8_t buf[108] __attribute__((aligned(16))) = {0};
    fnsave108(buf);

    /* fnstenv after fsave should report a freshly-initialized FPU. */
    uint8_t env[28] __attribute__((aligned(16))) = {0};
    fnstenv28(env);
    check_eq16("fsave-then-fnstenv: CW",  get16(env + 0), 0x037F);
    check_eq16("fsave-then-fnstenv: SW",  get16(env + 4), 0x0000);
    check_eq16("fsave-then-fnstenv: TW",  get16(env + 8), 0xFFFF);
}

/* ── Test 3: round-trip via fsave + frstor (both ours under JIT). ──────── */
static void test_round_trip(void) {
    const double vals[8] = {
        1.5,
        from_bits(0x4045000000000000ULL),  /* 42.0 */
        -1.0,
        2.0,
        -3.5,
        from_bits(0x7FF8000000000002ULL),  /* qNaN, different payload */
        100.25,
        -0.75,
    };

    fninit();
    for (int i = 0; i < 8; ++i) fld_double(&vals[i]);

    uint8_t buf[108] __attribute__((aligned(16))) = {0};
    fnsave108(buf);  /* state cleared */

    /* Scribble FPU state. */
    static const double scribble = 1234.5;
    fld_double(&scribble);
    fld_double(&scribble);

    /* Restore. */
    frstor108(buf);

    /* Pop ST(0..7) -> out[0..7]; out[i] = vals[7-i]. */
    double out[8];
    for (int i = 0; i < 8; ++i) fstp_double(&out[i]);

    char name[64];
    for (int i = 0; i < 8; ++i) {
        snprintf(name, sizeof(name), "fsave/frstor round-trip ST(%d) = vals[%d]", i, 7 - i);
        check_bits(name, out[i], vals[7 - i]);
    }
}

/* ── Test 4: fsave on an empty stack. ──────────────────────────────────── */
static void test_empty_stack(void) {
    fninit();  /* TOP=0, TW=0xFFFF */

    uint8_t buf[108] __attribute__((aligned(16))) = {0};
    fnsave108(buf);

    /* Verify the saved env reflects the empty stack. */
    check_eq16("fsave empty: saved CW",  get16(buf + 0), 0x037F);
    check_eq16("fsave empty: saved SW",  get16(buf + 4), 0x0000);
    check_eq16("fsave empty: saved TW",  get16(buf + 8), 0xFFFF);

    /* Post-state still re-initialized. */
    uint8_t env[28] __attribute__((aligned(16))) = {0};
    fnstenv28(env);
    check_eq16("fsave empty: post TW", get16(env + 8), 0xFFFF);
}

int main(void) {
    test_st_save();
    test_post_fsave_state();
    test_round_trip();
    test_empty_stack();

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
