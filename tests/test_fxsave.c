/*
 * test_fxsave.c — Tests for FXSAVE m512byte (0F AE /0).
 *
 * fxsave stores a 512-byte FPU+SSE state block. We don't translate this
 * opcode ourselves — it falls through the sidecar's nullopt path to stock
 * Rosetta, which handles the encoding (including the XMM area + MXCSR).
 *
 * We avoid asserting on the raw bytes of the saved area: Apple's Rosetta
 * uses an internal f80 representation that does not match Intel's
 * spec (verified empirically — `fxsave` of a `fld 1.5` writes
 * `00 00 00 00 00 00 00 c0 ff ff` for the ST(0) slot, which Intel would
 * read as -NaN). Testing the bytes against the Intel spec would give
 * spurious failures. Instead we test what the architecture guarantees:
 * fxsave + fxrstor must round-trip the FPU state, regardless of the
 * intermediate representation.
 *
 * What we DO assert:
 *   - The two architecturally documented header fields we actually
 *     touch from the JIT: FCW (offset 0), FSW.TOP (offset 2 bits 13:11),
 *     and the abridged FTW byte (offset 4) for empty/full stack.
 *   - End-to-end round-trip via fxsave → scribble → fxrstor.
 *   - fxsave does not modify FPU state (state preserved after save).
 *   - Composition: when JIT-handled FLDs precede fxsave, our run-end
 *     cache flush makes the X87State coherent in memory before stock's
 *     fxsave reads it; the round-trip still works.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void fninit(void)        { __asm__ volatile (".byte 0xDB, 0xE3"); }
static void fxsave_op(void *p)  { __asm__ volatile ("fxsave %0"  : "=m"(*(uint8_t*)p) :: "memory"); }
static void fxrstor_op(const void *p) { __asm__ volatile ("fxrstor %0" : : "m"(*(const uint8_t*)p) : "memory"); }

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

static void check_eq8(const char *name, uint8_t got, uint8_t expected) {
    if (got == expected) {
        printf("PASS  %s  (0x%02x)\n", name, got);
    } else {
        printf("FAIL  %s  got=0x%02x expected=0x%02x\n", name, got, expected);
        failures++;
    }
}

static uint16_t get16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }

/* ── Test 1: header fields are sensible after a known FPU state. ────────── */
static void test_header_fields(void) {
    fninit();
    static const double v = 1.0;
    /* Push 3 values onto the stack. After fninit (TOP=0), each fld
     * decrements TOP, so after 3 pushes TOP=5, slots 5/6/7 valid,
     * slots 0..4 empty.  abridged FTW = 0xE0 (physical R5/R6/R7 set;
     * abridged FTW is physical-indexed per Intel SDM 14.5.1.2). */
    fld_double(&v); fld_double(&v); fld_double(&v);

    uint8_t buf[512] __attribute__((aligned(16))) = {0};
    fxsave_op(buf);

    check_eq16("fxsave: FCW = 0x037F",        get16(buf + 0), 0x037F);
    check_eq16("fxsave: TOP = 5",             (get16(buf + 2) >> 11) & 7, 5);
    check_eq8 ("fxsave: abridged FTW = 0xE0", buf[4],          0xE0);

    fninit();
}

/* ── Test 2: fxsave does NOT reinitialize state. ─────────────────────────── */
static void test_post_fxsave_state(void) {
    fninit();
    static const double v[3] = {1.0, 2.0, 3.0};
    for (int i = 0; i < 3; ++i) fld_double(&v[i]);

    uint8_t buf[512] __attribute__((aligned(16))) = {0};
    fxsave_op(buf);

    /* ST values must still be on the stack — pop and verify. */
    double out[3];
    for (int i = 0; i < 3; ++i) fstp_double(&out[i]);

    check_bits("fxsave: post-state ST(0) preserved", out[0], v[2]);
    check_bits("fxsave: post-state ST(1) preserved", out[1], v[1]);
    check_bits("fxsave: post-state ST(2) preserved", out[2], v[0]);

    fninit();
}

/* ── Test 3: round-trip via fxsave + fxrstor (full 8-deep stack). ─────────
 * This is the strongest correctness test: regardless of what the
 * intermediate buffer encoding looks like, the FPU state must
 * round-trip bit-exactly.
 */
static void test_round_trip(void) {
    const double vals[8] = {
        1.5,
        from_bits(0x4045000000000000ULL),  /* 42.0 */
        -1.0,
        2.0,
        -3.5,
        from_bits(0x7FF0000000000000ULL),  /* +Inf */
        100.25,
        -0.75,
    };

    fninit();
    for (int i = 0; i < 8; ++i) fld_double(&vals[i]);

    uint8_t buf[512] __attribute__((aligned(16))) = {0};
    fxsave_op(buf);

    /* Scribble FPU state. */
    fninit();
    static const double scribble = 1234.5;
    fld_double(&scribble);
    fld_double(&scribble);

    /* Restore. */
    fxrstor_op(buf);

    /* Pop ST(0..7) -> out[0..7]; out[i] = vals[7-i]. */
    double out[8];
    for (int i = 0; i < 8; ++i) fstp_double(&out[i]);

    char name[64];
    for (int i = 0; i < 8; ++i) {
        snprintf(name, sizeof(name), "fxsave/fxrstor round-trip ST(%d)", i);
        check_bits(name, out[i], vals[7 - i]);
    }
}

/* ── Test 4: fxsave on an empty stack. ───────────────────────────────────── */
static void test_empty_stack(void) {
    fninit();  /* TOP=0, all slots empty */

    uint8_t buf[512] __attribute__((aligned(16))) = {0};
    fxsave_op(buf);

    check_eq16("fxsave empty: saved CW",       get16(buf + 0), 0x037F);
    check_eq16("fxsave empty: saved TOP",      (get16(buf + 2) >> 11) & 7, 0);
    check_eq8 ("fxsave empty: abridged FTW=0", buf[4],                    0x00);
}

/* ── Test 5: composition — JIT-handled FLDs precede fxsave/fxrstor.
 * Our JIT translates the 8 FLDs in a single x87 run; the run ends
 * before fxsave (which isn't in is_handled_x87), and `x87_end`
 * flushes deferred top/tag/perm state to memory so stock's fxsave
 * reads coherent X87State. After the round-trip the values must
 * still come back through JIT-handled fstp's. End-to-end check.
 */
static void test_composition_with_handled_ops(void) {
    const double vals[8] = {
        7.0, 14.0, 21.0, 28.0, 35.0, 42.0, 49.0, 56.0,
    };

    fninit();
    /* Each fld is JIT-handled. Form one x87 run with deferred state. */
    for (int i = 0; i < 8; ++i) fld_double(&vals[i]);

    /* fxsave is unhandled — sidecar nullopts → stub falls to STASH
     * → stock translates from coherent memory. */
    uint8_t buf[512] __attribute__((aligned(16))) = {0};
    fxsave_op(buf);

    /* abridged FTW must reflect the 8-slot push, not stale empty. */
    check_eq8("fxsave composition: abridged FTW after 8 FLDs",
              buf[4], 0xFF);
    check_eq16("fxsave composition: TOP=0 after 8 FLDs",
               (get16(buf + 2) >> 11) & 7, 0);

    /* Scribble + restore + pop via JIT-handled fstp's: round-trip
     * must hold end-to-end. */
    fninit();
    static const double scribble = -77.0;
    fld_double(&scribble);

    fxrstor_op(buf);

    double out[8];
    for (int i = 0; i < 8; ++i) fstp_double(&out[i]);

    char name[64];
    for (int i = 0; i < 8; ++i) {
        snprintf(name, sizeof(name),
                 "fxsave composition: ST(%d) round-trip", i);
        check_bits(name, out[i], vals[7 - i]);
    }
}

int main(void) {
    test_header_fields();
    test_post_fxsave_state();
    test_round_trip();
    test_empty_stack();
    test_composition_with_handled_ops();

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
