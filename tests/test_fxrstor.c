/*
 * test_fxrstor.c — Tests for FXRSTOR m512byte (0F AE /1).
 *
 * fxrstor reads a 512-byte FPU+SSE state block. We don't translate this
 * opcode ourselves — it falls through the sidecar's nullopt path to
 * stock Rosetta.
 *
 * We DON'T try to manually encode an Intel-spec f80 buffer: Apple's
 * Rosetta uses an internal representation that doesn't match Intel.
 * Instead, we let stock's `fxsave` produce a buffer (round-trip through
 * Apple's own encoding) and then exercise `fxrstor` on it. That keeps
 * the test independent of the buffer's byte format and verifies the
 * one architectural property we care about: an `fxsave/fxrstor` pair
 * preserves FPU state.
 *
 * This complements test_fxsave.c by emphasizing the *restore* side
 * and composition with JIT-handled ops that follow the fxrstor.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void fninit(void)             { __asm__ volatile (".byte 0xDB, 0xE3"); }
static void fxsave_op(void *p)       { __asm__ volatile ("fxsave %0"  : "=m"(*(uint8_t*)p) :: "memory"); }
static void fxrstor_op(const void *p) { __asm__ volatile ("fxrstor %0" : : "m"(*(const uint8_t*)p) : "memory"); }
static void fnstenv28(uint8_t *env)   { __asm__ volatile ("fnstenv %0" : "=m"(*env) :: "memory"); }

static void fld_double(const double *d) { __asm__ volatile ("fldl %0" : : "m"(*d)); }
static void fstp_double(double *d)      { __asm__ volatile ("fstpl %0" : "=m"(*d)); }

static uint64_t bits_of(double d) { uint64_t u; memcpy(&u, &d, sizeof(u)); return u; }
static double from_bits(uint64_t u) { double d; memcpy(&d, &u, sizeof(d)); return d; }

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

/* ── Test 1: round-trip 8 distinct ST values. ────────────────────────────── */
static void test_st_round_trip(void) {
    const double vals[8] = {
        1.0,
        2.5,
        from_bits(0x8000000000000000ULL),  /* -0.0 */
        from_bits(0x7FF0000000000000ULL),  /* +Inf */
        from_bits(0x7FF8000000000001ULL),  /* qNaN with payload */
        from_bits(0x4045000000000000ULL),  /* 42.0 */
        from_bits(0x7FEFFFFFFFFFFFFFULL),  /* MAX double */
        -42.5,
    };

    /* Build the buffer via fxsave so the byte format matches what
     * Apple's Rosetta expects fxrstor to read. */
    fninit();
    for (int i = 0; i < 8; ++i) fld_double(&vals[i]);

    uint8_t buf[512] __attribute__((aligned(16))) = {0};
    fxsave_op(buf);

    /* Scribble. */
    fninit();
    static const double scribble = 99.0;
    fld_double(&scribble); fld_double(&scribble); fld_double(&scribble);

    /* JIT path under test. */
    fxrstor_op(buf);

    /* Pop ST(0..7) -> out[0..7]; out[i] = vals[7-i]. */
    double out[8];
    for (int i = 0; i < 8; ++i) fstp_double(&out[i]);

    char name[64];
    for (int i = 0; i < 8; ++i) {
        snprintf(name, sizeof(name), "fxrstor: ST(%d) restored", i);
        check_bits(name, out[i], vals[7 - i]);
    }
}

/* ── Test 2: env round-trip — non-default CW + non-zero TOP via fxsave. ── */
static void test_env_round_trip(void) {
    /* Set up a non-default state, fxsave it, then verify fxrstor brings
     * it back via fnstenv. */
    static const double v[3] = {1.5, 2.5, 3.5};

    fninit();
    /* Configure CW to 0x0F7F (round-to-zero, all exceptions masked). */
    uint16_t cw = 0x0F7F;
    __asm__ volatile ("fldcw %0" : : "m"(cw));
    /* Push 3 values: TOP becomes 5. */
    for (int i = 0; i < 3; ++i) fld_double(&v[i]);

    uint8_t buf[512] __attribute__((aligned(16))) = {0};
    fxsave_op(buf);

    /* Reset, then fxrstor. */
    fninit();
    fxrstor_op(buf);

    uint8_t env[28] __attribute__((aligned(16))) = {0};
    fnstenv28(env);
    check_eq16("fxrstor env: CW restored to 0x0F7F", get16(env + 0), 0x0F7F);
    check_eq16("fxrstor env: TOP restored to 5",     (get16(env + 4) >> 11) & 7, 5);

    /* Drain. */
    double dummy;
    for (int i = 0; i < 3; ++i) fstp_double(&dummy);
    fninit();
}

/* ── Test 3: empty-stack round-trip. ─────────────────────────────────────── */
static void test_empty_stack(void) {
    fninit();  /* TOP=0, all empty */

    uint8_t buf[512] __attribute__((aligned(16))) = {0};
    fxsave_op(buf);

    /* Scribble. */
    static const double s = 1.0;
    fld_double(&s); fld_double(&s);

    fxrstor_op(buf);

    uint8_t env[28] __attribute__((aligned(16))) = {0};
    fnstenv28(env);
    check_eq16("fxrstor empty: TOP=0",     (get16(env + 4) >> 11) & 7, 0);
    check_eq16("fxrstor empty: TW=0xFFFF", get16(env + 8),             0xFFFF);
}

/* ── Test 4: composition — JIT-handled FLDs after fxrstor see restored
 * state.  fxsave produces the buffer; fxrstor then runs (stock-
 * translated); subsequent JIT-handled FSTPs must read the restored
 * X87State coherently. The cache must invalidate at the block boundary
 * so the JIT-handled run starts from a clean slate.
 */
static void test_composition_with_handled_ops(void) {
    const double vals[8] = {
        9.0, 18.0, 27.0, 36.0, 45.0, 54.0, 63.0, 72.0,
    };

    /* Build buffer via fxsave (Apple's encoding). */
    fninit();
    for (int i = 0; i < 8; ++i) fld_double(&vals[i]);
    uint8_t buf[512] __attribute__((aligned(16))) = {0};
    fxsave_op(buf);

    /* Run a small JIT-handled run to dirty the cache. */
    fninit();
    static const double sentinel = -1.0;
    fld_double(&sentinel);
    double sink;
    fstp_double(&sink);

    /* Stock-translated fxrstor — establishes ST(0..7). */
    fxrstor_op(buf);

    /* JIT-handled FSTPs in the next run must see the restored state. */
    double out[8];
    for (int i = 0; i < 8; ++i) fstp_double(&out[i]);

    char name[64];
    for (int i = 0; i < 8; ++i) {
        snprintf(name, sizeof(name),
                 "fxrstor composition: ST(%d) -> JIT pop", i);
        check_bits(name, out[i], vals[7 - i]);
    }
}

int main(void) {
    test_st_round_trip();
    test_env_round_trip();
    test_empty_stack();
    test_composition_with_handled_ops();

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
