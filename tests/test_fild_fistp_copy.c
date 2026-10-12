/*
 * test_fild_fistp_copy.c: fild m64 into fistp/fisttp m64 is an exact copy.
 *
 * x87 registers hold a 64-bit mantissa, so any int64 loaded with fild m64
 * stores back unchanged.  Delphi's System.Move copies memory this way, eight
 * bytes at a time, and eight ASCII bytes read as an int64 are around 2^62.
 * The sidecar holds x87 values as binary64, which rounds every |v| >= 2^53,
 * so these copies need the raw 64-bit path: a native transport reply for a
 * run of integer loads and stores only, the IR pass for longer runs mixing
 * in other ops, and the fild_fistp peephole for a lone or nested pair.
 *
 * Every check is bit-exact.  The cases cover the plain pair, one load
 * duplicated with fld st(0) and stored twice, overlapping moves in both
 * directions, a store into a source that was already read, a store that a
 * later fild reads back, a run mixing an exact copy with an fstpl, the
 * Delphi loop, and bridged mov gaps.  The last case puts an
 * fnstcw that writes the first source between that load and its store, so
 * a pressure relief that moved the load past the fnstcw would read the
 * control word (run_tests.sh's X87_FPR_POOL_LIMIT=5 phase asks for that
 * relief).
 *
 * Build: clang -arch x86_64 -O2 -o test_fild_fistp_copy test_fild_fistp_copy.c
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void check_i64(const char* name, int64_t got, int64_t expected) {
    if (got != expected) {
        printf("FAIL  %-52s  got=0x%016llx  expected=0x%016llx\n", name, (unsigned long long)got,
               (unsigned long long)expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

static void check_bytes(const char* name, const unsigned char* got, const unsigned char* expected,
                        int len) {
    for (int i = 0; i < len; i++) {
        if (got[i] != expected[i]) {
            printf("FAIL  %-52s  first difference at byte %d: got=0x%02x expected=0x%02x\n", name,
                   i, got[i], expected[i]);
            failures++;
            return;
        }
    }
    printf("PASS  %s\n", name);
}

static const int64_t kValues[] = {
    0x5357452E46535745LL,              /* "EWSF.EWS" */
    (1LL << 53) + 1,                   /* smallest value binary64 rounds */
    INT64_MAX,                         /* rounds to 2^63, out of range */
    INT64_MIN,                         /* exact in binary64 */
    -(1LL << 62) - 1,                  /* negative, rounds */
};
#define NVALUES ((int)(sizeof(kValues) / sizeof(kValues[0])))

/* ── Two-instruction runs (fild_fistp peephole) ──────────────────────────── */

static void copy_fistp(const int64_t* src, int64_t* dst) {
    __asm__ volatile(
        "fildll  (%1)\n\t"
        "fistpll (%0)\n"
        :
        : "r"(dst), "r"(src)
        : "memory");
}

static void copy_fisttp(const int64_t* src, int64_t* dst) {
    __asm__ volatile(
        "fildll   (%1)\n\t"
        "fisttpll (%0)\n"
        :
        : "r"(dst), "r"(src)
        : "memory");
}

/* ── Four-instruction runs (IR pass) ─────────────────────────────────────── */

/* Move 16 bytes the way Delphi does: both loads, then both stores, so the
 * copy is correct for overlapping buffers in either direction. */
static void move16(const int64_t* src, int64_t* dst) {
    __asm__ volatile(
        "fildll  8(%1)\n\t"
        "fildll  (%1)\n\t"
        "fistpll (%0)\n\t"
        "fistpll 8(%0)\n"
        :
        : "r"(dst), "r"(src)
        : "memory");
}

/* One load, duplicated with fld st(0) and stored twice: b = c = a. */
static void copy_twice(const int64_t* a, int64_t* b, int64_t* c) {
    __asm__ volatile(
        "fildll  (%0)\n\t"
        "fld     %%st(0)\n\t"
        "fistpll (%1)\n\t"
        "fistpll (%2)\n"
        :
        : "r"(a), "r"(b), "r"(c)
        : "memory");
}

/* Store into a source the run has already read: a = b, c = old a. */
static void store_into_read_source(int64_t* a, const int64_t* b, int64_t* c) {
    __asm__ volatile(
        "fildll  (%0)\n\t"
        "fildll  (%1)\n\t"
        "fistpll (%0)\n\t"
        "fistpll (%2)\n"
        :
        : "r"(a), "r"(b), "r"(c)
        : "memory");
}

/* Store, then load the stored value back: b = a, c = b. */
static void store_then_reload(const int64_t* a, int64_t* b, int64_t* c) {
    __asm__ volatile(
        "fildll  (%0)\n\t"
        "fistpll (%1)\n\t"
        "fildll  (%1)\n\t"
        "fistpll (%2)\n"
        :
        : "r"(a), "r"(b), "r"(c)
        : "memory");
}

/* One value copied exactly, the other stored as a double: c = a,
 * d = (double)b. */
static void copy_and_fstpl(const int64_t* a, const int64_t* b, int64_t* c, double* d) {
    __asm__ volatile(
        "fildll  (%0)\n\t"
        "fildll  (%1)\n\t"
        "fstpl   (%3)\n\t"
        "fistpll (%2)\n"
        :
        : "r"(a), "r"(b), "r"(c), "r"(d)
        : "memory");
}

/* ── The Delphi Move loop ────────────────────────────────────────────────── */

/* System.Move's large-block loop: a negative index counting up to zero over
 * the ends of both buffers, one fild/fistp pair per iteration. */
static void delphi_move40(const void* src, void* dst) {
    const char* s = (const char*)src + 40;
    char* d = (char*)dst + 40;
    int64_t n = -40;
    __asm__ volatile(
        "1:\n\t"
        "fildll  (%1,%0)\n\t"
        "fistpll (%2,%0)\n\t"
        "addq    $8, %0\n\t"
        "jl      1b\n"
        : "+r"(n)
        : "r"(s), "r"(d)
        : "memory", "cc");
}

/* ── Bridged runs ────────────────────────────────────────────────────────── */

/* Register moves between the loads and the stores: c = b, d = a. */
static uint32_t bridged_mov_gap(const int64_t* a, const int64_t* b, int64_t* c, int64_t* d) {
    uint32_t edx_out;
    __asm__ volatile(
        "fildll  (%1)\n\t"
        "fildll  (%2)\n\t"
        "movl    $0x1234beef, %%ecx\n\t" /* bridge: mov r,imm */
        "movl    %%ecx, %%edx\n\t"       /* bridge: mov r,r   */
        "fistpll (%3)\n\t"
        "fistpll (%4)\n\t"
        "movl    %%edx, %0\n"
        : "=r"(edx_out)
        : "r"(a), "r"(b), "r"(c), "r"(d)
        : "ecx", "edx", "memory");
    return edx_out;
}

/* A bridge store overwrites the first source after both loads: c = b,
 * d = old a, a = k. */
static void bridged_store_barrier(int64_t* a, const int64_t* b, int64_t* c, int64_t* d,
                                  int64_t k) {
    __asm__ volatile(
        "fildll  (%0)\n\t"
        "fildll  (%1)\n\t"
        "movq    %4, (%0)\n\t" /* bridge: mov [m],r */
        "fistpll (%2)\n\t"
        "fistpll (%3)\n"
        :
        : "r"(a), "r"(b), "r"(c), "r"(d), "r"(k)
        : "memory");
}

/* ── fnstcw between a load and its store, under pressure ─────────────────── */

/* c = old *a, out = {s[0], s[1], s[2] + s[3]}, and fnstcw overwrites the low
 * 16 bits of *a after the first load.  Five values are live at the faddp,
 * and no store comes between the first load and its fistpll, so on a
 * 5-register pool the cheapest relief is to load *a again just before that
 * fistpll, which would read the control word. */
static void fnstcw_between(int64_t* a, const int64_t* s, int64_t* c, int64_t* out) {
    __asm__ volatile(
        "fildll  (%0)\n\t"
        "fnstcw  (%0)\n\t"
        "fildll  (%1)\n\t"
        "fildll  8(%1)\n\t"
        "fildll  16(%1)\n\t"
        "fildll  24(%1)\n\t"
        "faddp\n\t"
        "fxch    %%st(3)\n\t"
        "fistpll (%2)\n\t"
        "fistpll 8(%3)\n\t"
        "fistpll (%3)\n\t"
        "fistpll 16(%3)\n"
        :
        : "r"(a), "r"(s), "r"(c), "r"(out)
        : "memory");
}

int main(void) {
    char name[96];

    for (int i = 0; i < NVALUES; i++) {
        const int64_t v = kValues[i];
        const int64_t w = kValues[(i + 1) % NVALUES];
        int64_t out = 0;

        copy_fistp(&v, &out);
        snprintf(name, sizeof(name), "fildll/fistpll   0x%016llx", (unsigned long long)v);
        check_i64(name, out, v);

        out = 0;
        copy_fisttp(&v, &out);
        snprintf(name, sizeof(name), "fildll/fisttpll  0x%016llx", (unsigned long long)v);
        check_i64(name, out, v);

        {
            /* dst = src + 8: the source tail is overwritten by the copy. */
            int64_t buf[3] = {v, w, 0};
            move16(&buf[0], &buf[1]);
            snprintf(name, sizeof(name), "move16 forward overlap [%d]", i);
            check_i64(name, buf[0], v);
            check_i64(name, buf[1], v);
            check_i64(name, buf[2], w);
        }
        {
            /* dst = src - 8. */
            int64_t buf[3] = {0, v, w};
            move16(&buf[1], &buf[0]);
            snprintf(name, sizeof(name), "move16 backward overlap [%d]", i);
            check_i64(name, buf[0], v);
            check_i64(name, buf[1], w);
            check_i64(name, buf[2], w);
        }
        {
            int64_t b = 0;
            int64_t c = 0;
            copy_twice(&v, &b, &c);
            snprintf(name, sizeof(name), "fld st(0), two stores b [%d]", i);
            check_i64(name, b, v);
            snprintf(name, sizeof(name), "fld st(0), two stores c [%d]", i);
            check_i64(name, c, v);
        }
        {
            int64_t a = v;
            int64_t c = 0;
            store_into_read_source(&a, &w, &c);
            snprintf(name, sizeof(name), "store into read source a [%d]", i);
            check_i64(name, a, w);
            snprintf(name, sizeof(name), "store into read source c [%d]", i);
            check_i64(name, c, v);
        }
        {
            int64_t b = 0;
            int64_t c = 0;
            store_then_reload(&v, &b, &c);
            snprintf(name, sizeof(name), "store then reload b [%d]", i);
            check_i64(name, b, v);
            snprintf(name, sizeof(name), "store then reload c [%d]", i);
            check_i64(name, c, v);
        }
        {
            int64_t c = 0;
            double d = 0.0;
            const double wd = (double)w;
            int64_t d_bits;
            int64_t wd_bits;
            copy_and_fstpl(&v, &w, &c, &d);
            memcpy(&d_bits, &d, sizeof(d));
            memcpy(&wd_bits, &wd, sizeof(wd));
            snprintf(name, sizeof(name), "copy beside fstpl, copy [%d]", i);
            check_i64(name, c, v);
            snprintf(name, sizeof(name), "copy beside fstpl, double [%d]", i);
            check_i64(name, d_bits, wd_bits);
        }
        {
            int64_t c = 0;
            int64_t d = 0;
            const uint32_t edx = bridged_mov_gap(&v, &w, &c, &d);
            snprintf(name, sizeof(name), "bridged mov gap c [%d]", i);
            check_i64(name, c, w);
            snprintf(name, sizeof(name), "bridged mov gap d [%d]", i);
            check_i64(name, d, v);
            snprintf(name, sizeof(name), "bridged mov gap edx [%d]", i);
            check_i64(name, edx, 0x1234beef);
        }
        {
            int64_t a = v;
            int64_t c = 0;
            int64_t d = 0;
            bridged_store_barrier(&a, &w, &c, &d, 0x0123456789abcdefLL);
            snprintf(name, sizeof(name), "bridged store barrier c [%d]", i);
            check_i64(name, c, w);
            snprintf(name, sizeof(name), "bridged store barrier d [%d]", i);
            check_i64(name, d, v);
            snprintf(name, sizeof(name), "bridged store barrier a [%d]", i);
            check_i64(name, a, 0x0123456789abcdefLL);
        }
    }

    {
        /* Five 8-byte ASCII chunks, "EWSF.EWS" at offset 8. */
        static const char src[41] = "GameDataEWSF.EWS_patch_1.0.0.42_NTDATA!!";
        unsigned char dst[40];
        memset(dst, 0, sizeof(dst));
        delphi_move40(src, dst);
        check_bytes("Delphi Move loop, 40 ASCII bytes", dst, (const unsigned char*)src, 40);
    }

    {
        /* Values below 2^53, so the result does not depend on whether the
         * run is split: only the order of the fnstcw and the first load
         * matters. */
        const int64_t first = 0x000123456789abcdLL;
        const int64_t s[4] = {11, -0x000fedcba9876543LL, 0x0000fedcba987654LL, 55};
        int64_t a = first;
        int64_t c = 0;
        int64_t out[3] = {0};
        uint16_t cw = 0;
        fnstcw_between(&a, s, &c, out);
        __asm__ volatile("fnstcw %0" : "=m"(cw));
        check_i64("fnstcw between: first load before fnstcw", c, first);
        check_i64("fnstcw between: fnstcw wrote the source", a,
                  (first & ~(int64_t)0xffff) | cw);
        check_i64("fnstcw between: out[0]", out[0], s[0]);
        check_i64("fnstcw between: out[1]", out[1], s[1]);
        check_i64("fnstcw between: out[2]", out[2], s[2] + s[3]);
    }

    printf("%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
