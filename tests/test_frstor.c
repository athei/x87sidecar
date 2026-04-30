/*
 * test_frstor.c — Tests for FRSTOR m108byte (DD /4).
 *
 * 32-bit protected-mode FPU state (108 bytes):
 *   +0..27   28-byte env header (CW/SW/TW + FIP/FCS/FOP/FDP/FDS)
 *   +28..107 8 ST slots in x86 f80 format, 10 bytes each:
 *     ST(0) @ 0x1C, ST(1) @ 0x26, ..., ST(7) @ 0x62
 *
 * The buffer is built manually using `long double` (which is f80 on x86_64)
 * rather than via FNSAVE — fsave is not yet inlined and falls through to stock
 * Rosetta, where the cache-boundary issue causes a crash.  Manual encoding
 * also makes the test independent of fsave correctness.
 *
 * Strategy:
 *   1. Encode 8 known doubles into a canonical 108-byte buffer (env + 8 f80).
 *   2. Scribble FPU state so any leftover ST values are obviously wrong.
 *   3. FRSTOR the buffer (this is the JIT path under test).
 *   4. Pop each ST slot with FSTP m64 and bit-exact compare.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void fninit(void)             { __asm__ volatile (".byte 0xDB, 0xE3"); }
static void frstor108(const uint8_t *buf) { __asm__ volatile ("frstor %0" : : "m"(*buf) : "memory"); }
static void fnstenv28(uint8_t *env)  { __asm__ volatile ("fnstenv %0" : "=m"(*env) :: "memory"); }

static void fstp_double(double *d)   { __asm__ volatile ("fstpl %0" : "=m"(*d)); }
static void fld1_op(void)            { __asm__ volatile ("fld1"); }

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

/* Encode a double into the 10-byte x86 f80 format using the host's long
 * double (which is f80 in x86_64 SysV).  This is the ground-truth encoding
 * we want frstor to read. */
static void encode_f80(uint8_t *out10, double d) {
    long double ld = (long double)d;
    memcpy(out10, &ld, 10);
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

static void put16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
static uint16_t get16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }

/* ── Test 1: round-trip 8 distinct ST values via a manually-built buffer. ── */
static void test_st_round_trip(void) {
    /* Distinct values that exercise sign / zero / inf / NaN / extreme exp. */
    const double vals[8] = {
        1.0,
        2.5,
        from_bits(0x8000000000000000ULL),  /* -0.0 */
        from_bits(0x7FF0000000000000ULL),  /* +Inf */
        from_bits(0x7FF8000000000001ULL),  /* qNaN with payload bit */
        from_bits(0x0010000000000000ULL),  /* MIN_NORMAL double */
        from_bits(0x7FEFFFFFFFFFFFFFULL),  /* MAX double */
        -42.5,
    };

    /* Build 108-byte buffer manually.
     *   CW = 0x037F (FPU default)
     *   SW = 0x0000 (TOP=0, no flags)
     *   TW = 0x0000 (all 8 slots valid)
     *   ST(0) @ 0x1C = vals[0], ..., ST(7) @ 0x62 = vals[7]
     */
    uint8_t buf[108] __attribute__((aligned(16))) = {0};
    put16(buf + 0, 0x037F);
    put16(buf + 4, 0x0000);
    put16(buf + 8, 0x0000);
    for (int i = 0; i < 8; ++i) encode_f80(buf + 0x1C + i * 10, vals[i]);

    /* Scribble FPU state. */
    fninit();
    fld1_op(); fld1_op();

    /* JIT path under test. */
    frstor108(buf);

    /* Pop ST(0..7) in order: ST(0)=vals[0], ..., ST(7)=vals[7]. */
    double out[8];
    for (int i = 0; i < 8; ++i) fstp_double(&out[i]);

    char name[64];
    for (int i = 0; i < 8; ++i) {
        snprintf(name, sizeof(name), "frstor: ST roundtrip vals[%d]", i);
        check_bits(name, out[i], vals[i]);
    }
}

/* ── Test 2: env round-trip — CW/SW/TW load and verify via fnstenv. ─────── */
static void test_env_round_trip(void) {
    /* Build a buffer with non-default env: CW=0x0F7F (round-to-zero, all
     * exceptions masked), TOP=3, TW marks slots 5,6,7 as valid. */
    uint8_t buf[108] __attribute__((aligned(16))) = {0};
    put16(buf + 0, 0x0F7F);
    put16(buf + 4, (uint16_t)(3 << 11));    /* TOP = 3 */
    put16(buf + 8, 0x03FF);                  /* slots 5,6,7 valid (bits 10..15 = 00 00 00) */
    /* Fill slots 5,6,7 (= phys (3+5)&7=0, (3+6)&7=1, (3+7)&7=2) with 1.5/2.5/3.5. */
    encode_f80(buf + 0x1C + 5 * 10, 1.5);
    encode_f80(buf + 0x1C + 6 * 10, 2.5);
    encode_f80(buf + 0x1C + 7 * 10, 3.5);
    /* Other slots: leave as 0 bytes (all-zero f80 = +0.0). */

    /* Scribble. */
    fninit();
    fld1_op();

    frstor108(buf);

    uint8_t env[28] __attribute__((aligned(16))) = {0};
    fnstenv28(env);
    check_eq16("frstor env: CW",  get16(env + 0), 0x0F7F);
    check_eq16("frstor env: TOP", (get16(env + 4) >> 11) & 7, 3);
    check_eq16("frstor env: TW",  get16(env + 8), 0x03FF);

    /* Drain the 3 valid slots so we don't leak FPU stack into other tests. */
    double dummy;
    fstp_double(&dummy);  /* ST(5) — wait, but ST(5) means logical ST(5), so we
                           * need 5 fincstp pops first, or just finit. */
    fninit();
}

/* ── Test 3: empty stack — TW=0xFFFF, TOP=0. ───────────────────────────── */
static void test_empty_stack(void) {
    uint8_t buf[108] __attribute__((aligned(16))) = {0};
    put16(buf + 0, 0x037F);
    put16(buf + 4, 0x0000);   /* TOP=0 */
    put16(buf + 8, 0xFFFF);   /* all empty */

    /* Scribble. */
    fld1_op(); fld1_op();

    frstor108(buf);

    uint8_t env[28] __attribute__((aligned(16))) = {0};
    fnstenv28(env);
    check_eq16("frstor empty: TOP=0",      (get16(env + 4) >> 11) & 7, 0);
    check_eq16("frstor empty: TW=0xFFFF",  get16(env + 8), 0xFFFF);

    fninit();
}

int main(void) {
    test_st_round_trip();
    test_env_round_trip();
    test_empty_stack();

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
