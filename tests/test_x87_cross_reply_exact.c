/*
 * test_x87_cross_reply_exact.c: an 80-bit value held on the x87 stack while
 * other blocks do x87 arithmetic must come out bit for bit.
 *
 * Each case loads a value in one block (fld m80 or fild m64), runs a second
 * block of x87 work that never touches it, and stores it in a third block
 * (fstp m80 or fistp m64).  Real x87 and stock Rosetta keep every 80-bit
 * register as it is, so the stored bytes equal the loaded ones: 64-bit
 * integers of 2^53 or more, exponents outside binary64's range, all NaN
 * payload bits, unnormals and denormals.  A translator that converts every
 * register to binary64 at each reply rounds them, which is what
 * X87_NATIVE_SLOTS avoids: there each register is converted only where a
 * reply reads it.
 *
 * The middle block takes the shapes the translator emits differently: an
 * IR run (direct emitters and fusions when the harness turns the IR off),
 * single-instruction replies, two- and three-instruction fusions, an IR run
 * bridged across a mov, transcendentals including the pushing fxtract,
 * fsincos and fptan, fcmov, deferred FXCH, ffree, fincstp and fdecstp inside
 * a run, and pushes that leave the held value at every depth from 1 to 7.
 * Further middles read the held value without changing it, which the
 * translator does from the 80-bit image (fld st(i), fst and fstp to a
 * register, m64 and m32, fist and fistp, fnsave and frstor; fst m32, fist
 * and fistp also inside a run the IR lowers), and move it between physical
 * registers through a deferred FXCH.  What they read must
 * equal the same read done in the loading block, and fnsave must save the
 * held value's ten bytes; --dump prints those reads so a run under stock
 * Rosetta can be compared with one under x87sidecar.  --int-stores-may-round
 * accepts a 64-bit integer store rounded to 53 bits, for a harness run that
 * leaves the translator too few registers for its exact integer store.  Every case also runs
 * with TOP rotated by fdecstp before the load, so the held value sits in
 * each of the eight physical registers and the middle block's pushes wrap
 * TOP past 0.
 * Blocks are separated by `jmp 1f; 1:`, which ends a translation block.
 *
 * Expected: passes under stock Rosetta and under x87sidecar, fails with
 * X87_NATIVE_SLOTS=0 (run_tests.sh checks both).
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

typedef struct {
    uint64_t mant;
    uint16_t sexp;
} __attribute__((packed)) f80;

typedef struct {
    f80 v;
    uint8_t pad[6];
} __attribute__((aligned(16))) f80_slot;

static const double kConst[] = {3.0, 2.0, 0.25, 1.0, 0.5};

#define SEP "jmp 1f\n1:\n\t"
#define CLOBBERS                                                                             \
    "memory", "cc", "rax", "st", "st(1)", "st(2)", "st(3)", "st(4)", "st(5)", "st(6)", "st(7)"

/* ── Middle blocks ───────────────────────────────────────────────────────── */
/* Each pushes and pops only its own values, writing results to out[]. */

#define MID_IR                                                                   \
    "fldl 0(%[c])\n\tfmull 8(%[c])\n\tfaddl 16(%[c])\n\tfsqrt\n\tfstpl 0(%[out])\n\t"
#define MID_SINGLE "fldl 0(%[c])\n\t" SEP "fstpl 0(%[out])\n\t"
#define MID_FUSE2 "fldl 0(%[c])\n\tfstpl 0(%[out])\n\t"
#define MID_FUSE3 "fldl 0(%[c])\n\tfmull 8(%[c])\n\tfstpl 0(%[out])\n\t"
#define MID_FSIN "fldl 32(%[c])\n\tfsin\n\tfstpl 0(%[out])\n\t"
#define MID_FYL2X "fld1\n\tfldl 8(%[c])\n\tfyl2x\n\tfstpl 0(%[out])\n\t"
#define MID_FCMOV                                                                       \
    "cmpl %k[z], %k[z]\n\tfldl 0(%[c])\n\tfldl 8(%[c])\n\tfcmove %%st(1), %%st\n\t"     \
    "fstpl 0(%[out])\n\tfstpl 8(%[out])\n\t"
#define MID_FXCH1                                                                       \
    "fldl 0(%[c])\n\tfldl 8(%[c])\n\tfxch %%st(1)\n\tfstpl 0(%[out])\n\tfstpl 8(%[out])\n\t"
#define MID_FXCH2                                                                       \
    "fldl 0(%[c])\n\tfldl 8(%[c])\n\tfldl 16(%[c])\n\tfxch %%st(2)\n\tfstpl 0(%[out])\n\t" \
    "fstpl 8(%[out])\n\tfstpl 16(%[out])\n\t"
/* The mov joins the x87 instructions on either side into one bridged IR run. */
#define MID_BRIDGE "fldl 0(%[c])\n\tmovl $5, %%eax\n\tfmull 8(%[c])\n\tfstpl 0(%[out])\n\t"
#define MID_FXTRACT "fldl 0(%[c])\n\tfxtract\n\tfaddp\n\tfstpl 0(%[out])\n\t"
#define MID_FSINCOS "fldl 32(%[c])\n\tfsincos\n\tfaddp\n\tfstpl 0(%[out])\n\t"
#define MID_FPTAN "fldl 32(%[c])\n\tfptan\n\tfaddp\n\tfstpl 0(%[out])\n\t"
/* ffree, fincstp and fdecstp inside a run: the freed 2.0 is skipped. */
#define MID_TOPMOVE                                                                      \
    "fldl 0(%[c])\n\tfldl 8(%[c])\n\tffree %%st(0)\n\tfincstp\n\tfdecstp\n\tfincstp\n\t" \
    "fstpl 0(%[out])\n\t"
/* Middles that read the held value, or move it, and leave it as it was. */
#define MID_RD_FLDCOPY "fld %%st(0)\n\tfmull 8(%[c])\n\tfstpl 0(%[out])\n\t"
#define MID_RD_FST64 "fstl 0(%[out])\n\t"
#define MID_RD_FST32 "fsts 0(%[out])\n\t"
#define MID_RD_FSTST "fldz\n\tfld %%st(1)\n\tfstp %%st(1)\n\tfstpl 0(%[out])\n\t"
#define MID_RD_FIST "fistl 0(%[out])\n\t"
#define MID_RD_FISTP "fld %%st(0)\n\tfistpll 0(%[out])\n\t"
#define MID_RD_FSAVE "fnsave 0(%[out])\n\tfrstor 0(%[out])\n\t"
/* The same reads inside a run long enough for the IR. */
#define MID_RD_IRFST32 "fldz\n\tfstpl 8(%[out])\n\tfsts 0(%[out])\n\t"
#define MID_RD_IRFIST "fldz\n\tfstpl 8(%[out])\n\tfistl 0(%[out])\n\t"
#define MID_RD_IRFISTP "fldz\n\tfstpl 8(%[out])\n\tfld %%st(0)\n\tfistpll 0(%[out])\n\t"
/* The held value changes physical register twice through a deferred FXCH
 * flushed at a push and at a pop. */
#define MID_RD_FXCH                                                              \
    "fld1\n\tfxch %%st(1)\n\tfldz\n\tfstp %%st(0)\n\tfxch %%st(1)\n\tfstpl 0(%[out])\n\t"
#define PUSH1 "fldl 24(%[c])\n\t"
#define ADDP "faddp\n\t"
#define MID_D1 PUSH1 "fstpl 0(%[out])\n\t"
#define MID_D2 PUSH1 PUSH1 ADDP "fstpl 0(%[out])\n\t"
#define MID_D3 PUSH1 PUSH1 PUSH1 ADDP ADDP "fstpl 0(%[out])\n\t"
#define MID_D4 PUSH1 PUSH1 PUSH1 PUSH1 ADDP ADDP ADDP "fstpl 0(%[out])\n\t"
#define MID_D5 PUSH1 PUSH1 PUSH1 PUSH1 PUSH1 ADDP ADDP ADDP ADDP "fstpl 0(%[out])\n\t"
#define MID_D6 PUSH1 PUSH1 PUSH1 PUSH1 PUSH1 PUSH1 ADDP ADDP ADDP ADDP ADDP "fstpl 0(%[out])\n\t"
#define MID_D7                                                                    \
    PUSH1 PUSH1 PUSH1 PUSH1 PUSH1 PUSH1 PUSH1 ADDP ADDP ADDP ADDP ADDP ADDP \
    "fstpl 0(%[out])\n\t"

/* ── Load and store forms ────────────────────────────────────────────────── */

enum { kM80, kI64, kI64ToM80 };
#define LOAD_M80 "fldt (%[src])\n\t"
#define STORE_M80 "fstpt (%[dst])\n"
#define LOAD_I64 "fildll (%[src])\n\t"
#define STORE_I64 "fistpll (%[dst])\n"

typedef void (*case_fn)(const void* src, void* dst, double* out);

#define DEFINE_CASE(name, LOAD, MID, STORE)                                                  \
    static void name(const void* src, void* dst, double* out) {                              \
        __asm__ volatile(LOAD SEP MID SEP STORE                                              \
                         :                                                                   \
                         : [src] "r"(src), [dst] "r"(dst), [out] "r"(out), [c] "r"(kConst), \
                           [z] "r"(0)                                                        \
                         : CLOBBERS);                                                        \
    }

#define DEFINE_FORMS(mid)                                    \
    DEFINE_CASE(m80_##mid, LOAD_M80, MID_##mid, STORE_M80)   \
    DEFINE_CASE(i64_##mid, LOAD_I64, MID_##mid, STORE_I64)   \
    DEFINE_CASE(i64m80_##mid, LOAD_I64, MID_##mid, STORE_M80)

/* References for the read-only middles: the same read in the block that
 * loads the value, so no reply boundary lies between them. */
typedef void (*ref_fn)(const void* src, double* out);

#define REF_RD_FLDCOPY "fld %%st(0)\n\tfmull 8(%[c])\n\tfstpl 0(%[out])\n\tfstp %%st(0)\n\t"
#define REF_RD_FST64 "fstl 0(%[out])\n\tfstp %%st(0)\n\t"
#define REF_RD_FST32 "fsts 0(%[out])\n\tfstp %%st(0)\n\t"
#define REF_RD_FSTST "fstpl 0(%[out])\n\t"
#define REF_RD_FIST "fistl 0(%[out])\n\tfstp %%st(0)\n\t"
#define REF_RD_FISTP "fistpll 0(%[out])\n\t"
#define REF_RD_IRFST32 REF_RD_FST32
#define REF_RD_IRFIST REF_RD_FIST
#define REF_RD_IRFISTP REF_RD_FISTP

#define DEFINE_REF(name, LOAD, BODY)                                                       \
    static void name(const void* src, double* out) {                                       \
        __asm__ volatile(LOAD BODY                                                         \
                         :                                                                 \
                         : [src] "r"(src), [out] "r"(out), [c] "r"(kConst)                 \
                         : CLOBBERS);                                                      \
    }

#define DEFINE_REFS(mid)                                \
    DEFINE_REF(ref_m80_##mid, LOAD_M80, REF_##mid)      \
    DEFINE_REF(ref_i64_##mid, LOAD_I64, REF_##mid)

DEFINE_REFS(RD_FLDCOPY)
DEFINE_REFS(RD_FST64)
DEFINE_REFS(RD_FST32)
DEFINE_REFS(RD_FSTST)
DEFINE_REFS(RD_FIST)
DEFINE_REFS(RD_FISTP)
DEFINE_REFS(RD_IRFST32)
DEFINE_REFS(RD_IRFIST)
DEFINE_REFS(RD_IRFISTP)

DEFINE_FORMS(IR)
DEFINE_FORMS(SINGLE)
DEFINE_FORMS(FUSE2)
DEFINE_FORMS(FUSE3)
DEFINE_FORMS(FSIN)
DEFINE_FORMS(FYL2X)
DEFINE_FORMS(FCMOV)
DEFINE_FORMS(FXCH1)
DEFINE_FORMS(FXCH2)
DEFINE_FORMS(RD_FLDCOPY)
DEFINE_FORMS(RD_FST64)
DEFINE_FORMS(RD_FST32)
DEFINE_FORMS(RD_FSTST)
DEFINE_FORMS(RD_FIST)
DEFINE_FORMS(RD_FISTP)
DEFINE_FORMS(RD_FSAVE)
DEFINE_FORMS(RD_IRFST32)
DEFINE_FORMS(RD_IRFIST)
DEFINE_FORMS(RD_IRFISTP)
DEFINE_FORMS(RD_FXCH)
DEFINE_FORMS(BRIDGE)
DEFINE_FORMS(FXTRACT)
DEFINE_FORMS(FSINCOS)
DEFINE_FORMS(FPTAN)
DEFINE_FORMS(TOPMOVE)
DEFINE_FORMS(D1)
DEFINE_FORMS(D2)
DEFINE_FORMS(D3)
DEFINE_FORMS(D4)
DEFINE_FORMS(D5)
DEFINE_FORMS(D6)
DEFINE_FORMS(D7)

typedef struct {
    const char* name;
    case_fn fn[3];   /* indexed by kM80, kI64, kI64ToM80 */
    int nout;        /* results to check */
    double want[3];  /* exact results, NAN-free; a 0 count skips the check */
    /* Read-only middles: out[] bytes [read_off, read_off + read_len) must
     * equal what ref (indexed by kM80, kI64) writes there, or with no ref,
     * the held value's ten bytes (fnsave's ST(0)). */
    ref_fn ref[2];
    int read_off;
    int read_len;
    int int_read; /* the read is an integer store */
} middle;

#define FORMS(mid) {m80_##mid, i64_##mid, i64m80_##mid}
#define REFS(mid) {ref_m80_##mid, ref_i64_##mid}

static const middle kMiddles[] = {
    {"ir", FORMS(IR), 1, {2.5}},
    {"single-op", FORMS(SINGLE), 1, {3.0}},
    {"fusion2", FORMS(FUSE2), 1, {3.0}},
    {"fusion3", FORMS(FUSE3), 1, {6.0}},
    {"fsin", FORMS(FSIN), 0, {0}},
    {"fyl2x", FORMS(FYL2X), 0, {0}},
    {"fcmove", FORMS(FCMOV), 2, {3.0, 3.0}},
    {"fxch st1", FORMS(FXCH1), 2, {3.0, 2.0}},
    {"fxch st2", FORMS(FXCH2), 3, {3.0, 2.0, 0.25}},
    {"reads: fld st(0) copy", FORMS(RD_FLDCOPY), 0, {0}, REFS(RD_FLDCOPY), 0, 8},
    {"reads: fst m64", FORMS(RD_FST64), 0, {0}, REFS(RD_FST64), 0, 8},
    {"reads: fst m32", FORMS(RD_FST32), 0, {0}, REFS(RD_FST32), 0, 4},
    {"reads: fld st(1), fstp st(1)", FORMS(RD_FSTST), 0, {0}, REFS(RD_FSTST), 0, 8},
    {"reads: fist m32", FORMS(RD_FIST), 0, {0}, REFS(RD_FIST), 0, 4},
    {"reads: fld st(0), fistp m64", FORMS(RD_FISTP), 0, {0}, REFS(RD_FISTP), 0, 8, 1},
    {"reads: fnsave, frstor", FORMS(RD_FSAVE), 0, {0}, {0, 0}, 28, 10},
    {"reads in a run: fst m32", FORMS(RD_IRFST32), 0, {0}, REFS(RD_IRFST32), 0, 4},
    {"reads in a run: fist m32", FORMS(RD_IRFIST), 0, {0}, REFS(RD_IRFIST), 0, 4},
    {"reads in a run: fld st(0), fistp m64", FORMS(RD_IRFISTP), 0, {0}, REFS(RD_IRFISTP), 0, 8, 1},
    {"moves: fxch across a push and a pop", FORMS(RD_FXCH), 1, {1.0}},
    {"bridged", FORMS(BRIDGE), 1, {6.0}},
    {"fxtract", FORMS(FXTRACT), 1, {2.5}},
    {"fsincos", FORMS(FSINCOS), 0, {0}},
    {"fptan", FORMS(FPTAN), 0, {0}},
    {"ffree fincstp fdecstp", FORMS(TOPMOVE), 1, {3.0}},
    {"depth1", FORMS(D1), 1, {1.0}},
    {"depth2", FORMS(D2), 1, {2.0}},
    {"depth3", FORMS(D3), 1, {3.0}},
    {"depth4", FORMS(D4), 1, {4.0}},
    {"depth5", FORMS(D5), 1, {5.0}},
    {"depth6", FORMS(D6), 1, {6.0}},
    {"depth7", FORMS(D7), 1, {7.0}},
};

/* ── TOP rotation ────────────────────────────────────────────────────────── */

static void rotate_down(int n) {
    for (int i = 0; i < n; i++) {
        __asm__ volatile("fdecstp" ::: "memory");
    }
}

static void rotate_up(int n) {
    for (int i = 0; i < n; i++) {
        __asm__ volatile("fincstp" ::: "memory");
    }
}

static uint16_t status_word(void) {
    uint16_t sw;
    __asm__ volatile("fnstsw %0" : "=m"(sw));
    return sw;
}

/* ── Values ──────────────────────────────────────────────────────────────── */

typedef struct {
    const char* name;
    f80 v;
} f80_value;

static const f80_value kF80Values[] = {
    {"2^62+3", {0x8000000000000006ULL, 0x403D}},
    {"INT64_MIN", {0x8000000000000000ULL, 0xC03E}},
    {"NaN with low payload bits", {0xC000000000000801ULL, 0x7FFF}},
    {"about 1e4000", {0xDE3A1BB4C0F8A971ULL, 0x73E6}},
    {"unnormal", {0x4000000000000C01ULL, 0x4005}},
    {"unnormal above binary64's range", {0x4000000000000001ULL, 0x5000}},
    {"pseudo-NaN", {0x4000000000000001ULL, 0x7FFF}},
    {"denormal", {0x0000000000123457ULL, 0x0000}},
    {"negative 1 + 2^-63", {0x8000000000000001ULL, 0xBFFF}},
    /* fst m32 rounds this up to 1 + 2^-23; through binary64 it would become
     * the tie 1 + 2^-24 and round to 1. */
    {"1 + 2^-24 + 2^-60", {0x8000008000000008ULL, 0x3FFF}},
};

typedef struct {
    const char* name;
    int64_t v;
    f80 as_f80;
} i64_value;

static const i64_value kI64Values[] = {
    {"2^62+3", 4611686018427387907LL, {0x8000000000000006ULL, 0x403D}},
    {"INT64_MIN", INT64_MIN, {0x8000000000000000ULL, 0xC03E}},
    {"2^53+1", 9007199254740993LL, {0x8000000000000400ULL, 0x4034}},
    {"-(2^63-1)", -INT64_MAX, {0xFFFFFFFFFFFFFFFEULL, 0xC03D}},
};

/* ── Runner ──────────────────────────────────────────────────────────────── */

static int reported = 0;

static void report(const char* what, const char* mid, int rot, const char* val, const void* got,
                   const void* want, size_t n) {
    failures++;
    if (++reported > 40) {
        return;
    }
    const uint8_t* g = got;
    const uint8_t* w = want;
    printf("FAIL  %s, middle %s, TOP rotated %d, %s: got ", what, mid, rot, val);
    for (size_t i = n; i-- > 0;) {
        printf("%02x", g[i]);
    }
    printf(" want ");
    for (size_t i = n; i-- > 0;) {
        printf("%02x", w[i]);
    }
    printf("\n");
}

static void check_middle(const middle* m, int rot, const double* out) {
    for (int i = 0; i < m->nout; i++) {
        if (memcmp(&out[i], &m->want[i], sizeof(double)) != 0) {
            report("middle result", m->name, rot, "-", &out[i], &m->want[i], sizeof(double));
        }
    }
}

static int dump = 0;

/* --int-stores-may-round: the harness has left the translator too few
 * registers for its exact integer store, which then reads binary64.  A
 * 64-bit integer store may give the value rounded to 53 bits. */
static int int_stores_may_round = 0;

static int64_t round_to_53_bits(int64_t v) {
    if (v == INT64_MIN) {
        return v;
    }
    uint64_t mag = v < 0 ? (uint64_t)-v : (uint64_t)v;
    const int width = 64 - __builtin_clzll(mag | 1);
    if (width <= 53) {
        return v;
    }
    const int shift = width - 53;
    const uint64_t rem = mag & ((1ULL << shift) - 1);
    const uint64_t half = 1ULL << (shift - 1);
    mag -= rem;
    if (rem > half || (rem == half && ((mag >> shift) & 1) != 0)) {
        mag += 1ULL << shift;
    }
    return v < 0 ? -(int64_t)mag : (int64_t)mag;
}

static int int64_matches(const void* got, const void* want) {
    int64_t g;
    int64_t w;
    memcpy(&g, got, sizeof(g));
    memcpy(&w, want, sizeof(w));
    /* Either side may have rounded (a reference read too). */
    return g == w || (int_stores_may_round && round_to_53_bits(g) == round_to_53_bits(w));
}

static void check_read(const middle* m, int form, int rot, const char* val, const void* src,
                       const f80* held, const double* out) {
    if (m->read_len == 0) {
        return;
    }
    double ref[16] = {0};
    const uint8_t* want = (const uint8_t*)held;
    if (m->ref[form] != NULL) {
        m->ref[form](src, ref);
        want = (const uint8_t*)ref + m->read_off;
    }
    const uint8_t* got = (const uint8_t*)out + m->read_off;
    const int same = m->int_read && m->read_len == 8 ? int64_matches(got, want)
                                                     : memcmp(got, want, (size_t)m->read_len) == 0;
    if (!same) {
        report("read of the held value", m->name, rot, val, got, want, (size_t)m->read_len);
    }
    if (dump && rot == 0) {
        printf("DUMP  %s | %s | %s | ", m->name, form == kM80 ? "m80" : "i64", val);
        for (int i = m->read_len; i-- > 0;) {
            printf("%02x", got[i]);
        }
        printf("\n");
    }
}

static void check_balanced(const middle* m, int rot) {
    const uint16_t sw = status_word();
    const uint16_t top = (sw >> 11) & 7;
    if (top != 0) {
        const uint16_t zero = 0;
        report("TOP after the case", m->name, rot, "-", &top, &zero, sizeof(top));
    }
}

int main(int argc, char** argv) {
    /* --dump prints what the read-only middles read, to compare runs. */
    for (int i = 1; i < argc; i++) {
        dump |= strcmp(argv[i], "--dump") == 0;
        int_stores_may_round |= strcmp(argv[i], "--int-stores-may-round") == 0;
    }
    int cases = 0;
    for (size_t mi = 0; mi < sizeof(kMiddles) / sizeof(kMiddles[0]); mi++) {
        const middle* m = &kMiddles[mi];
        for (int rot = 0; rot < 8; rot++) {
            for (size_t vi = 0; vi < sizeof(kF80Values) / sizeof(kF80Values[0]); vi++) {
                f80_slot src;
                f80_slot dst;
                double out[16] = {0};  /* fnsave writes 108 bytes */
                memset(&src, 0, sizeof(src));
                memset(&dst, 0xA5, sizeof(dst));
                src.v = kF80Values[vi].v;
                rotate_down(rot);
                m->fn[kM80](&src, &dst, out);
                rotate_up(rot);
                cases++;
                if (memcmp(&dst.v, &src.v, sizeof(f80)) != 0) {
                    report("fld m80 to fstp m80", m->name, rot, kF80Values[vi].name, &dst.v,
                           &src.v, sizeof(f80));
                }
                check_middle(m, rot, out);
                check_read(m, kM80, rot, kF80Values[vi].name, &src, &src.v, out);
                check_balanced(m, rot);
            }
            for (size_t vi = 0; vi < sizeof(kI64Values) / sizeof(kI64Values[0]); vi++) {
                const int64_t src = kI64Values[vi].v;
                int64_t dst = 0x5A5A5A5A5A5A5A5ALL;
                double out[16] = {0};  /* fnsave writes 108 bytes */
                rotate_down(rot);
                m->fn[kI64](&src, &dst, out);
                rotate_up(rot);
                cases++;
                if (!int64_matches(&dst, &src)) {
                    report("fild m64 to fistp m64", m->name, rot, kI64Values[vi].name, &dst, &src,
                           sizeof(dst));
                }
                check_middle(m, rot, out);
                check_read(m, kI64, rot, kI64Values[vi].name, &src, &kI64Values[vi].as_f80, out);
                check_balanced(m, rot);

                f80_slot d80;
                memset(&d80, 0xA5, sizeof(d80));
                rotate_down(rot);
                m->fn[kI64ToM80](&src, &d80, out);
                rotate_up(rot);
                cases++;
                if (memcmp(&d80.v, &kI64Values[vi].as_f80, sizeof(f80)) != 0) {
                    report("fild m64 to fstp m80", m->name, rot, kI64Values[vi].name, &d80.v,
                           &kI64Values[vi].as_f80, sizeof(f80));
                }
                check_balanced(m, rot);
            }
        }
    }
    if (failures == 0) {
        printf("PASS  %d cases: values held across replies come out bit for bit\n", cases);
    }
    printf("%d failures\n", failures);
    return failures != 0;
}
