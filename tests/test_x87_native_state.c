/* Native save areas must contain real f80 values, including across x87 runs. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures;

static uint64_t g_random = 0x123456789abcdef;
static uint64_t next_random(void) {
    g_random ^= g_random << 13;
    g_random ^= g_random >> 7;
    g_random ^= g_random << 17;
    return g_random;
}

/* The f80 an int64 loads as: exact, normalised, +0 for zero. */
static void i64_to_f80(int64_t v, unsigned char out[10]) {
    uint64_t mant = 0;
    uint16_t se = 0;
    if (v != 0) {
        const uint64_t a = v < 0 ? 0 - (uint64_t)v : (uint64_t)v;
        const int lz = __builtin_clzll(a);
        mant = a << lz;
        se = (uint16_t)((v < 0 ? 0x8000 : 0) | (16383 + 63 - lz));
    }
    memcpy(out, &mant, 8);
    memcpy(out + 8, &se, 2);
}

static int64_t int64_pattern(unsigned i) {
    static const int64_t edges[] = {
        0,
        1,
        -1,
        (1LL << 53) - 1,
        (1LL << 53) + 1,
        -(1LL << 53) - 1,
        (1LL << 62) + 1,
        INT64_MAX,
        INT64_MIN,
        INT64_MIN + 1,
        0x5357452E46535745LL,
    };
    if (i < sizeof(edges) / sizeof(edges[0]))
        return edges[i];
    /* Random magnitudes across every exponent, both signs. */
    const uint64_t r = next_random();
    return (int64_t)(r >> (next_random() & 63));
}

/* fild leaves an exact f80 in the native register file, as stock's fxsave
 * sees it from a later block. */
static void test_fild_payload(void) {
    int bad = 0;
    for (unsigned i = 0; i < 4096; ++i) {
        const int64_t v = int64_pattern(i);
        unsigned char state[512] __attribute__((aligned(16)));
        unsigned char want[10];
        __asm__ volatile("fninit\n\tfildll %1\n\tjmp 1f\n1:\n\tfxsave %0\n\tfninit"
                         : "=m"(state)
                         : "m"(v)
                         : "memory", "st");
        i64_to_f80(v, want);
        if (memcmp(state + 32, want, 10) != 0) {
            if (bad++ < 5) {
                uint64_t m;
                uint16_t e;
                memcpy(&m, state + 32, 8);
                memcpy(&e, state + 40, 2);
                printf("FAIL  fild payload v=%016llx got=%04x:%016llx\n", (unsigned long long)v,
                       e, (unsigned long long)m);
            }
        }
    }
    printf("%s  fild, jmp, fxsave: exact f80 for 4096 int64 patterns\n", bad ? "FAIL" : "PASS");
    failures += bad != 0;
}

/* fild in one block, fistp in the next. */
static void test_fild_fistp_blocks(void) {
    int bad = 0;
    g_random = 0x2545f4914f6cdd1dULL;
    for (unsigned i = 0; i < 4096; ++i) {
        const int64_t v = int64_pattern(i);
        int64_t got = 0;
        __asm__ volatile("fildll %1\n\tjmp 1f\n1:\n\tfistpll %0" : "=m"(got) : "m"(v) : "st");
        if (got != v && bad++ < 5)
            printf("FAIL  fild, jmp, fistp v=%016llx got=%016llx\n", (unsigned long long)v,
                   (unsigned long long)got);
    }
    printf("%s  fild, jmp, fistp: 4096 int64 patterns\n", bad ? "FAIL" : "PASS");
    failures += bad != 0;
}

/* ── Integer stores of any f80 ───────────────────────────────────────────── */

static uint64_t indefinite(int width) {
    return width == 16 ? 0x8000U : width == 32 ? 0x80000000U : 0x8000000000000000ULL;
}

/* The integer an x87 store writes for the f80 (mant, se) under rounding
 * control rc (3 = chop), as the low `width` bits.  The value is taken in
 * 64.64 fixed point, so this does not share the translator's shift split. */
static uint64_t ref_store(uint64_t mant, uint16_t se, int rc, int width) {
    const int e = se & 0x7fff;
    const int neg = se >> 15;
    if (e == 0x7fff || (e != 0 && !(mant >> 63)))
        return indefinite(width); /* NaN, infinity, unnormal */
    uint64_t ip = 0;
    uint64_t frac = 0;
    if (e >= 16383 + 64) {
        return indefinite(width); /* |v| >= 2^64 */
    } else if (e >= 16383) {
        /* v = mant * 2^(e - 16383 - 63), a 64.64 value of mant << (e - 16383 + 1) */
        const unsigned __int128 fixed = (unsigned __int128)mant << (e - 16383 + 1);
        ip = (uint64_t)(fixed >> 64);
        frac = (uint64_t)fixed;
    } else if (e >= 16383 - 64) {
        ip = 0;
        frac = mant >> (16383 - 1 - e);
        if (e < 16382 && (mant << (64 - (16383 - 1 - e))) != 0)
            frac |= 1; /* sticky, never reaches one half */
        if (e < 16382 && frac == 0 && mant != 0)
            frac = 1;
    } else {
        frac = mant != 0; /* far below one half, denormals included */
    }
    int up = 0;
    if (rc == 0)
        up = frac > 0x8000000000000000ULL || (frac == 0x8000000000000000ULL && (ip & 1));
    else if (rc == 1)
        up = neg && frac != 0;
    else if (rc == 2)
        up = !neg && frac != 0;
    if (up) {
        if (ip == UINT64_MAX)
            return indefinite(width);
        ip++;
    }
    /* In range: -2^(w-1) <= v <= 2^(w-1) - 1. */
    const uint64_t lim = 1ULL << (width - 1);
    if (neg ? ip > lim : ip >= lim)
        return indefinite(width);
    const uint64_t mask = width == 64 ? ~0ULL : (1ULL << width) - 1;
    return (neg ? 0 - ip : ip) & mask;
}

struct f80 {
    uint64_t mant;
    uint16_t se;
};

static struct f80 f80_pattern(unsigned i) {
    static const struct f80 edges[] = {
        {0, 0},                                          /* +0 */
        {0, 0x8000},                                     /* -0 */
        {0x8000000000000000ULL, 0x3ffe},                 /* 0.5 */
        {0x8000000000000000ULL, 0xbffe},                 /* -0.5 */
        {0xc000000000000000ULL, 0x3fff},                 /* 1.5 */
        {0xa000000000000000ULL, 0x4000},                 /* 2.5 */
        {0xa000000000000000ULL, 0xc000},                 /* -2.5 */
        {0x8000000000000001ULL, 0x3ffe},                 /* just above 0.5 */
        {0xffffffffffffffffULL, 0x3ffd},                 /* just below 0.5 */
        {0x8000000000000001ULL, 0x403d},                 /* 2^62 + 0.5, a 64-bit tie */
        {0x8000000000000003ULL, 0x403d},                 /* 2^62 + 1.5 */
        {0xffffffffffffffffULL, 0x403d},                 /* 2^63 - 0.5 */
        {0xffffffffffffffffULL, 0xc03d},                 /* -(2^63 - 0.5) */
        {0x8000000000000000ULL, 0x403e},                 /* 2^63 */
        {0x8000000000000000ULL, 0xc03e},                 /* -2^63 */
        {0x8000000000000001ULL, 0xc03e},                 /* below -2^63 */
        {0xfffffffffffffffeULL, 0x403e},                 /* 2^64 - 2 */
        {0x8000000000000000ULL, 0x7fff},                 /* +inf */
        {0x8000000000000000ULL, 0xffff},                 /* -inf */
        {0xc000000000000000ULL, 0x7fff},                 /* qNaN */
        {0xfffffffe00000000ULL, 0x401d},                 /* 2^31 - 1 */
        {0xffffffff00000000ULL, 0x401d},                 /* 2^31 - 0.5 */
        {0x8000000000000000ULL, 0xc01e},                 /* -2^31 */
        {0xfffe000000000000ULL, 0x400d},                 /* 32767 */
        {0xffff000000000000ULL, 0x400d},                 /* 32767.5 */
        {0x8000800000000000ULL, 0xc00e},                 /* -32768.5 */
        {0x8000000000000000ULL, 0x0001},                 /* smallest normal */
        {0x8000000000000000ULL, 0x8001},                 /* its negative */
        {0x0000000000000001ULL, 0x0000},                 /* smallest denormal */
        {0x0000000000000001ULL, 0x8000},                 /* its negative */
        {0x4000000000000000ULL, 0x3fff},                 /* unnormal */
        {0x8000000000000000ULL, 0x0000},                 /* pseudo-denormal */
        {0x0000000000000000ULL, 0x4010},                 /* pseudo-zero */
    };
    const unsigned n = sizeof(edges) / sizeof(edges[0]);
    if (i < n)
        return edges[i];
    /* Random normals with exponents from 2^-70 to 2^65, both signs. */
    struct f80 r;
    r.mant = next_random() | 0x8000000000000000ULL;
    const uint64_t s = next_random();
    r.se = (uint16_t)((16383 - 70 + (int)(s % 136)) | ((s >> 32) & 1 ? 0x8000 : 0));
    return r;
}

/* Load one f80 as ST(0) through stock's fxrstor, then store it as an
 * integer in the next block under rounding control rc. */
static void load_f80(unsigned char* state, struct f80 v, int rc) {
    uint16_t fcw = (uint16_t)(0x037f | (rc << 10));
    uint16_t fsw = 7 << 11; /* TOP = 7 */
    memcpy(state + 0, &fcw, 2);
    memcpy(state + 2, &fsw, 2);
    state[4] = 0x80; /* abridged tag: physical register 7 valid */
    memset(state + 32, 0, 128);
    memcpy(state + 32, &v.mant, 8);
    memcpy(state + 40, &v.se, 2);
}

static void test_f80_integer_stores(void) {
    int bad = 0;
    int cases = 0;
    unsigned char state[512] __attribute__((aligned(16)));
    __asm__ volatile("fninit\n\tfxsave %0" : "=m"(state));
    g_random = 0x9e3779b97f4a7c15ULL;
    for (unsigned i = 0; i < 1024; ++i) {
        const struct f80 v = f80_pattern(i);
        for (int rc = 0; rc < 4; ++rc) {
            int64_t s64 = 0, t64 = 0;
            int32_t s32 = 0, f32 = 0;
            int16_t s16 = 0;
            load_f80(state, v, rc);
            __asm__ volatile("fxrstor %1\n\tjmp 1f\n1:\n\tfistpll %0" : "=m"(s64) : "m"(state) : "st");
            __asm__ volatile("fxrstor %1\n\tjmp 1f\n1:\n\tfistpl %0" : "=m"(s32) : "m"(state) : "st");
            __asm__ volatile("fxrstor %1\n\tjmp 1f\n1:\n\tfistps %0" : "=m"(s16) : "m"(state) : "st");
            __asm__ volatile("fxrstor %1\n\tjmp 1f\n1:\n\tfistl %0\n\tjmp 2f\n2:\n\tfstp %%st(0)"
                             : "=m"(f32)
                             : "m"(state)
                             : "st");
            __asm__ volatile("fxrstor %1\n\tjmp 1f\n1:\n\tfisttpll %0" : "=m"(t64) : "m"(state) : "st");
            __asm__ volatile("fninit");
            const uint64_t got[5] = {(uint64_t)s64, (uint32_t)s32, (uint16_t)s16, (uint32_t)f32,
                                     (uint64_t)t64};
            const uint64_t want[5] = {ref_store(v.mant, v.se, rc, 64),
                                      ref_store(v.mant, v.se, rc, 32),
                                      ref_store(v.mant, v.se, rc, 16),
                                      ref_store(v.mant, v.se, rc, 32),
                                      ref_store(v.mant, v.se, 3, 64)};
            static const char* const names[5] = {"fistp m64", "fistp m32", "fistp m16",
                                                 "fist m32", "fisttp m64"};
            for (int k = 0; k < 5; ++k) {
                cases++;
                if (got[k] != want[k] && bad++ < 10)
                    printf("FAIL  %s rc=%d f80=%04x:%016llx got=%llx want=%llx\n", names[k], rc,
                           v.se, (unsigned long long)v.mant, (unsigned long long)got[k],
                           (unsigned long long)want[k]);
            }
        }
    }
    printf("%s  integer stores of native f80 values, every RC (%d cases, %d bad)\n",
           bad ? "FAIL" : "PASS", cases, bad);
    failures += bad != 0;
}

static void test_boundary_bits(void) {
    const uint64_t cases[] = {
        0,
        1,
        2,
        0x000fffffffffffff,
        0x0010000000000000,
        0x8000000000000001,
        0x800fffffffffffff,
        0x8000000000000000,
        0x3ff0000000000000,
        0x7fefffffffffffff,
        0x7ff0000000000000,
        0xfff0000000000000,
        0x7ff8000000000001,
        0xffffffffffffffff,
    };
    int bad = 0;
    uint64_t random = 0x123456789abcdef;
    for (unsigned i = 0; i < 4096; ++i) {
        random ^= random << 13;
        random ^= random >> 7;
        random ^= random << 17;
        uint64_t bits = i < sizeof(cases) / sizeof(cases[0]) ? cases[i] : random;
        /* Quiet NaNs: a load may quiet a signaling NaN architecturally. */
        if ((bits & UINT64_C(0x7fffffffffffffff)) > UINT64_C(0x7ff0000000000000))
            bits |= UINT64_C(0x0008000000000000);
        uint64_t got;
        __asm__ volatile("fldl %1; jmp 1f; 1: fstpl %0" : "=m"(got) : "m"(bits) : "st");
        if (got != bits) {
            if (bad++ < 5)
                printf("FAIL  boundary bits in=%016llx out=%016llx\n", bits, got);
        }
    }
    printf("%s  native boundary: 4096 binary64 bit patterns\n", bad ? "FAIL" : "PASS");
    failures += bad != 0;
}

static void test_extended_rounding(void) {
    const struct {
        uint64_t mant;
        uint16_t exp;
        uint64_t expected;
    } cases[] = {
        {0x8000000000000400, 0x3fff, 0x3ff0000000000000},  // even tie
        {0x8000000000000c00, 0x3fff, 0x3ff0000000000002},  // odd tie
        {0x8000000000000000, 0x3bcc, 0},                   // half minimum subnormal
        {0x8000000000000001, 0x3bcc, 1},
        {0x8000000000000000, 0xbbcc, 0x8000000000000000},
        {0xffffffffffffffff, 0x43fe, 0x7ff0000000000000},  // rounded overflow
        {0x8000000000000000, 0x43ff, 0x7ff0000000000000},  // exponent overflow
        {0x8000000000000001, 0x7fff, 0x7ff8000000000000},  // low NaN payload
    };
    int bad = 0;
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        unsigned char native[10];
        memcpy(native, &cases[i].mant, 8);
        memcpy(native + 8, &cases[i].exp, 2);
        uint64_t got;
        __asm__ volatile("fldt %1; jmp 1f; 1: fstpl %0" : "=m"(got) : "m"(native) : "st");
        if (got != cases[i].expected) {
            printf("FAIL  extended rounding case=%u got=%016llx expected=%016llx\n", i, got,
                   cases[i].expected);
            bad++;
        }
    }
    printf("%s  extended rounding: ties, underflow, overflow and NaN\n", bad ? "FAIL" : "PASS");
    failures += bad != 0;
}

static void test_fxsave_payload(void) {
    unsigned char state[512] __attribute__((aligned(16)));
    const double values[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    double restored[8];
    __asm__ volatile(
        "fninit\n"
        "fldl 0(%1); fldl 8(%1); fldl 16(%1); fldl 24(%1)\n"
        "fldl 32(%1); fldl 40(%1); fldl 48(%1); fldl 56(%1)\n"
        "fxsave %0\nfninit"
        : "=m"(state)
        : "r"(values)
        : "memory", "st", "st(1)", "st(2)", "st(3)", "st(4)", "st(5)", "st(6)", "st(7)");
    int bad = 0;
    for (unsigned i = 0; i < 8; ++i) {
        long double expected = values[7 - i];
        if (memcmp(state + 32 + 16 * i, &expected, 10)) {
            printf("FAIL  fxsave native payload slot=%u\n", i);
            bad++;
        }
        /* Edit every slot to test native input independently of our output. */
        long double replacement = values[i] + 0.25L;
        memcpy(state + 32 + 16 * i, &replacement, 10);
    }
    __asm__ volatile(
        "fxrstor %1\n"
        "fstpl 0(%0); fstpl 8(%0); fstpl 16(%0); fstpl 24(%0)\n"
        "fstpl 32(%0); fstpl 40(%0); fstpl 48(%0); fstpl 56(%0)"
        :
        : "r"(restored), "m"(state)
        : "memory", "st", "st(1)", "st(2)", "st(3)", "st(4)", "st(5)", "st(6)", "st(7)");
    for (unsigned i = 0; i < 8; ++i)
        if (restored[i] != values[i] + 0.25) {
            printf("FAIL  fxrstor native payload slot=%u got=%g\n", i, restored[i]);
            bad++;
        }
    printf("%s  native save/restore: all eight slots\n", bad ? "FAIL" : "PASS");
    failures += bad != 0;
}

int main(void) {
    test_boundary_bits();
    test_extended_rounding();
    test_fxsave_payload();
    test_fild_payload();
    test_fild_fistp_blocks();
    test_f80_integer_stores();
    return failures ? 1 : 0;
}
