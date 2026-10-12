/*
 * test_delphi_move.c: Delphi's System.Move must copy bytes exactly.
 *
 * The Delphi RTL copies 9 to 32 bytes, and the head and tail of any larger
 * block, by loading 8-byte chunks with fild m64 and storing them with fistp
 * m64.  The chunks stay on the x87 stack across the routine's conditional
 * branches and loops, so each fistp usually sits in a different basic block,
 * and so in a different translation, from the fild that loaded its value.
 * An x87 register holds any int64 exactly; a translation that keeps x87
 * values as binary64 between blocks rounds every chunk of 2^53 or more, and
 * eight ASCII bytes read as an int64 are around 2^62.  The Delphi game
 * NosTale lost file paths this way.
 *
 * delphi_move below is the RTL routine with its 32-bit registers widened to
 * 64 bits: the 9..32 byte path, the forward loop and the backward loop.  The
 * 0..8 byte path is a jump table of integer moves and is not used here.
 * Every check compares bytes against memmove, including guard bytes around
 * the destination.  The cases cover ASCII paths, int64 edge values and random
 * data, misaligned and overlapping buffers in both directions, every rounding
 * and precision control setting (fild and fistp ignore both), the minimal
 * two-block shapes, Delphi's Round and Trunc shapes, and a SIGUSR1 storm in
 * two variants: a handler that only counts and one that writes the saved
 * context back unchanged.
 *
 * NosTale itself is 32-bit code, so the 9..32 byte path and the two-block
 * Round shape also run unwidened in a 32-bit compatibility segment from the
 * LDT, as in test_x87_signal_context.  The image is linked below 4 GB so the
 * 32-bit code can address its buffers.  The large-block loops run in the
 * widened routine only.
 *
 * Build: clang -arch x86_64 -O2 -Wl,-pagezero_size,0x4000 -o test_delphi_move \
 *        test_delphi_move.c
 */
#include <architecture/i386/desc.h>
#include <architecture/i386/table.h>
#include <errno.h>
#include <i386/user_ldt.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static int failures = 0;

/* ── System.Move ─────────────────────────────────────────────────────────── */

/* rax = source, rdx = destination, rcx = count, as eax/edx/ecx in the RTL. */
void delphi_move(const void* src, void* dst, int64_t count);
__asm__(
    ".text\n"
    ".p2align 4\n"
    ".globl _delphi_move\n"
    "_delphi_move:\n"
    ".intel_syntax noprefix\n"
    "    mov     rax, rdi\n"
    "    mov     rcx, rdx\n"
    "    mov     rdx, rsi\n"
    "    cmp     rax, rdx\n"
    "    je      Ldm_exit\n"
    "    cmp     rcx, 32\n"
    "    ja      Ldm_large\n"
    "    sub     rcx, 8\n"
    "    jg      Ldm_small\n"
    "    ret\n" /* 0..8 bytes: the RTL's jump table, not used by this test */
    "Ldm_small:\n"
    "    fild    qword ptr [rax+rcx]\n" /* last 8 */
    "    fild    qword ptr [rax]\n"     /* first 8 */
    "    cmp     rcx, 8\n"
    "    jle     Ldm_small16\n"
    "    fild    qword ptr [rax+8]\n" /* second 8 */
    "    cmp     rcx, 16\n"
    "    jle     Ldm_small24\n"
    "    fild    qword ptr [rax+16]\n"  /* third 8 */
    "    fistp   qword ptr [rdx+16]\n"
    "Ldm_small24:\n"
    "    fistp   qword ptr [rdx+8]\n"
    "Ldm_small16:\n"
    "    fistp   qword ptr [rdx]\n"
    "    fistp   qword ptr [rdx+rcx]\n"
    "Ldm_exit:\n"
    "    ret\n"
    "Ldm_forward:\n"
    "    push    rdx\n"
    "    fild    qword ptr [rax]\n" /* first 8 */
    "    lea     rax, [rax+rcx-8]\n"
    "    lea     rcx, [rcx+rdx-8]\n"
    "    fild    qword ptr [rax]\n" /* last 8 */
    "    push    rcx\n"
    "    neg     rcx\n"
    "    and     rdx, -8\n" /* 8-byte aligned writes */
    "    lea     rcx, [rcx+rdx+8]\n"
    "    pop     rdx\n"
    "Ldm_fwd_loop:\n"
    "    fild    qword ptr [rax+rcx]\n"
    "    fistp   qword ptr [rdx+rcx]\n"
    "    add     rcx, 8\n"
    "    jl      Ldm_fwd_loop\n"
    "    fistp   qword ptr [rdx]\n" /* last 8 */
    "    pop     rdx\n"
    "    fistp   qword ptr [rdx]\n" /* first 8 */
    "    ret\n"
    "Ldm_large:\n"
    "    jng     Ldm_large_done\n" /* count < 0 */
    "    cmp     rax, rdx\n"
    "    ja      Ldm_forward\n"
    "    sub     rdx, rcx\n"
    "    cmp     rax, rdx\n"
    "    lea     rdx, [rdx+rcx]\n"
    "    jna     Ldm_forward\n"
    "    sub     rcx, 8\n" /* backward move */
    "    push    rcx\n"
    "    fild    qword ptr [rax+rcx]\n" /* last 8 */
    "    fild    qword ptr [rax]\n"     /* first 8 */
    "    add     rcx, rdx\n"
    "    and     rcx, -8\n" /* 8-byte aligned writes */
    "    sub     rcx, rdx\n"
    "Ldm_bwd_loop:\n"
    "    fild    qword ptr [rax+rcx]\n"
    "    fistp   qword ptr [rdx+rcx]\n"
    "    sub     rcx, 8\n"
    "    jg      Ldm_bwd_loop\n"
    "    pop     rcx\n"
    "    fistp   qword ptr [rdx]\n"     /* first 8 */
    "    fistp   qword ptr [rdx+rcx]\n" /* last 8 */
    "Ldm_large_done:\n"
    "    ret\n"
    ".att_syntax prefix\n");

/* ── Shorter shapes ──────────────────────────────────────────────────────── */

/* fild in one block, fistp in the next. */
static void two_block(const int64_t* s, int64_t* d, int64_t c) {
    __asm__ volatile(
        "fildll  (%0)\n\t"
        "cmpq    $8, %2\n\t"
        "jle     1f\n"
        "1:\n\t"
        "fistpll (%1)\n"
        :
        : "r"(s), "r"(d), "r"(c)
        : "memory", "cc");
}

/* Round(): the value reaches the RTL's fistp from another block. */
static void round_i64(const int64_t* s, int64_t* d) {
    __asm__ volatile(
        "fildll  (%0)\n\t"
        "jmp     1f\n"
        "1:\n\t"
        "fistpll (%1)\n"
        :
        : "r"(s), "r"(d)
        : "memory");
}

static void round_d64(const double* s, int64_t* d) {
    __asm__ volatile("fldl (%0)\n\tjmp 1f\n1:\n\tfistpll (%1)\n" : : "r"(s), "r"(d) : "memory");
}

static void round_d32(const double* s, int32_t* d) {
    __asm__ volatile("fldl (%0)\n\tjmp 1f\n1:\n\tfistpl (%1)\n" : : "r"(s), "r"(d) : "memory");
}

static void round_d16(const double* s, int16_t* d) {
    __asm__ volatile("fldl (%0)\n\tjmp 1f\n1:\n\tfistps (%1)\n" : : "r"(s), "r"(d) : "memory");
}

/* fist without the pop; the value is dropped in a later block. */
static void fist_d32(const double* s, int32_t* d) {
    __asm__ volatile("fldl (%0)\n\tjmp 1f\n1:\n\tfistl (%1)\n\tjmp 2f\n2:\n\tfstp %%st(0)\n"
                     :
                     : "r"(s), "r"(d)
                     : "memory");
}

static void fist_d16(const double* s, int16_t* d) {
    __asm__ volatile("fldl (%0)\n\tjmp 1f\n1:\n\tfists (%1)\n\tjmp 2f\n2:\n\tfstp %%st(0)\n"
                     :
                     : "r"(s), "r"(d)
                     : "memory");
}

static void fisttp_d64(const double* s, int64_t* d) {
    __asm__ volatile("fldl (%0)\n\tjmp 1f\n1:\n\tfisttpll (%1)\n" : : "r"(s), "r"(d) : "memory");
}

static void fisttp_d32(const double* s, int32_t* d) {
    __asm__ volatile("fldl (%0)\n\tjmp 1f\n1:\n\tfisttpl (%1)\n" : : "r"(s), "r"(d) : "memory");
}

static void fisttp_d16(const double* s, int16_t* d) {
    __asm__ volatile("fldl (%0)\n\tjmp 1f\n1:\n\tfisttps (%1)\n" : : "r"(s), "r"(d) : "memory");
}

/* Trunc(): save the control word, set RC to chop and PC to 64 bits, store,
 * restore.  The fldcw / fistp / fldcw run follows an integer `or`. */
static void trunc_d64(const double* s, int64_t* d) {
    uint16_t cw[2];
    __asm__ volatile(
        "fldl    (%0)\n\t"
        "jmp     1f\n"
        "1:\n\t"
        "fnstcw  (%2)\n\t"
        "fnstcw  2(%2)\n\t"
        "orw     $0x0f00, 2(%2)\n\t"
        "fldcw   2(%2)\n\t"
        "fistpll (%1)\n\t"
        "fldcw   (%2)\n"
        :
        : "r"(s), "r"(d), "r"(cw)
        : "memory", "cc");
}

static void set_cw(uint16_t cw) {
    __asm__ volatile("fldcw %0" : : "m"(cw));
}

#define CW_DEFAULT 0x037f

/* ── Data ────────────────────────────────────────────────────────────────── */

#define MAXN 1000
#define GUARD 24

static const char* const kPaths[] = {
    "C:\\Games\\NostaleData\\EWSF.EWS",
    "C:\\Program Files (x86)\\GameforgeLive\\Games\\DEU_ger\\NosTale\\NostaleData\\NStcData.NOS",
    "D:\\NosTale\\NostaleData\\NSgtdData.NOS",
};

static const int64_t kSpecials[] = {
    INT64_MAX,
    INT64_MIN,
    -1,
    (1LL << 53) - 1,
    (1LL << 53) + 1, /* 0x0020000000000001, the smallest value binary64 rounds */
    (1LL << 54) + 3,
    -(1LL << 53) - 1,
    0x5357452E46535745LL, /* "EWSF.EWS" */
    (1LL << 62) + 1,
    -(1LL << 62) - 1,
    INT64_MAX - 1,
    INT64_MIN + 1,
    0,
    1,
};
#define NSPECIALS ((int)(sizeof(kSpecials) / sizeof(kSpecials[0])))

static uint64_t g_rng = 0x9e3779b97f4a7c15ULL;
static uint64_t next_random(void) {
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 7;
    g_rng ^= g_rng << 17;
    return g_rng;
}

enum pattern { P_PATH, P_SPECIAL, P_RANDOM, P_COUNT };
static const char* const kPatternNames[] = {"ascii path", "int64 edges", "random"};

/* Fill len bytes for a pattern; `phase` varies which bytes land where. */
static void fill(unsigned char* p, int len, int pattern, int phase) {
    if (pattern == P_PATH) {
        const char* s = kPaths[phase % 3];
        const int n = (int)strlen(s);
        for (int i = 0; i < len; i++) {
            p[i] = (unsigned char)s[(i + phase) % n];
        }
    } else if (pattern == P_SPECIAL) {
        for (int i = 0; i < len; i += 8) {
            const int64_t v = kSpecials[(i / 8 + phase) % NSPECIALS];
            const int n = len - i < 8 ? len - i : 8;
            memcpy(p + i, &v, (size_t)n);
        }
    } else {
        for (int i = 0; i < len; i++) {
            p[i] = (unsigned char)(next_random() >> 24);
        }
    }
}

static const int kCounts[] = {9, 15, 16, 17, 24, 25, 32, 33, 40, 64, 100, 1000};
#define NCOUNTS ((int)(sizeof(kCounts) / sizeof(kCounts[0])))

/* ── Checking ────────────────────────────────────────────────────────────── */

static int report_mismatch(const char* what, const unsigned char* got, const unsigned char* want,
                           int len) {
    for (int i = 0; i < len; i++) {
        if (got[i] != want[i]) {
            printf("FAIL  %s  first difference at byte %d of %d: got=0x%02x want=0x%02x\n", what,
                   i, len, got[i], want[i]);
            return 1;
        }
    }
    return 0;
}

/* Separate buffers: src at offset so, dst at offset do. */
static int move_separate(int count, int pattern, int phase, int so, int dso) {
    static unsigned char src[MAXN + 64];
    static unsigned char dst[MAXN + 2 * GUARD + 16];
    static unsigned char want[MAXN + 2 * GUARD + 16];
    char what[160];
    memset(src, 0x55, sizeof(src));
    fill(src + so, count, pattern, phase);
    memset(dst, 0xAA, sizeof(dst));
    memcpy(want, dst, sizeof(want));
    memcpy(want + GUARD + dso, src + so, (size_t)count);
    delphi_move(src + so, dst + GUARD + dso, count);
    snprintf(what, sizeof(what), "Move count=%d %s src+%d dst+%d", count, kPatternNames[pattern],
             so, dso);
    return report_mismatch(what, dst, want, count + 2 * GUARD + dso);
}

/* One buffer, destination `shift` bytes after (positive) or before
 * (negative) the source. */
static int move_overlap(int count, int pattern, int phase, int shift) {
    static unsigned char buf[2 * MAXN + 4 * GUARD];
    static unsigned char want[2 * MAXN + 4 * GUARD];
    char what[160];
    const int src_off = 2 * GUARD + (shift < 0 ? -shift : 0);
    const int dst_off = src_off + shift;
    const int total = count + 4 * GUARD + (shift < 0 ? -shift : shift);
    memset(buf, 0xAA, sizeof(buf));
    fill(buf + src_off, count, pattern, phase);
    memcpy(want, buf, sizeof(want));
    memmove(want + dst_off, want + src_off, (size_t)count);
    delphi_move(buf + src_off, buf + dst_off, count);
    snprintf(what, sizeof(what), "Move count=%d %s overlap shift=%d", count,
             kPatternNames[pattern], shift);
    return report_mismatch(what, buf, want, total);
}

static void test_move_separate(void) {
    static const int kOffsets[][2] = {{0, 0}, {1, 0}, {0, 3}, {5, 7}, {7, 1}, {4, 4}};
    for (int p = 0; p < P_COUNT; p++) {
        int bad = 0;
        int cases = 0;
        for (int c = 0; c < NCOUNTS; c++) {
            for (unsigned o = 0; o < sizeof(kOffsets) / sizeof(kOffsets[0]); o++) {
                for (int phase = 0; phase < 3; phase++) {
                    bad += move_separate(kCounts[c], p, phase, kOffsets[o][0], kOffsets[o][1]);
                    cases++;
                }
            }
        }
        printf("%s  Move separate buffers, %s (%d cases, %d bad)\n", bad ? "FAIL" : "PASS",
               kPatternNames[p], cases, bad);
        failures += bad != 0;
    }
}

static void test_move_overlap(void) {
    static const int kShifts[] = {1, 8, 13, 31, -1, -8, -13, -31};
    for (int p = 0; p < P_COUNT; p++) {
        int bad = 0;
        int cases = 0;
        for (int c = 0; c < NCOUNTS; c++) {
            for (unsigned s = 0; s < sizeof(kShifts) / sizeof(kShifts[0]); s++) {
                bad += move_overlap(kCounts[c], p, (int)s, kShifts[s]);
                cases++;
            }
        }
        printf("%s  Move overlapping, both directions, %s (%d cases, %d bad)\n",
               bad ? "FAIL" : "PASS", kPatternNames[p], cases, bad);
        failures += bad != 0;
    }
}

/* fild and fistp ignore both rounding and precision control. */
static void test_move_control_word(void) {
    static const int kPc[] = {0, 2, 3}; /* 24, 53 and 64 bits */
    int bad = 0;
    int cases = 0;
    for (int rc = 0; rc < 4; rc++) {
        for (int pc = 0; pc < 3; pc++) {
            const uint16_t cw = (uint16_t)(0x007f | (kPc[pc] << 8) | (rc << 10));
            for (int c = 0; c < NCOUNTS; c++) {
                for (int p = 0; p < P_COUNT; p++) {
                    set_cw(cw);
                    bad += move_separate(kCounts[c], p, c, c & 7, (c * 3) & 7);
                    bad += move_overlap(kCounts[c], p, c, (c & 1) ? 13 : -13);
                    set_cw(CW_DEFAULT);
                    cases += 2;
                }
            }
        }
    }
    printf("%s  Move under every RC and PC setting (%d cases, %d bad)\n", bad ? "FAIL" : "PASS",
           cases, bad);
    failures += bad != 0;
}

static void test_two_block(void) {
    int bad = 0;
    for (int i = 0; i < NSPECIALS; i++) {
        for (int64_t c = 1; c <= 9; c += 8) { /* jle taken, then not taken */
            int64_t d = 0;
            two_block(&kSpecials[i], &d, c);
            if (d != kSpecials[i]) {
                printf("FAIL  two-block fild/jle/fistp 0x%016llx ecx=%lld got=0x%016llx\n",
                       (unsigned long long)kSpecials[i], (long long)c, (unsigned long long)d);
                bad++;
            }
        }
    }
    printf("%s  two-block fild, jle, fistp (%d values)\n", bad ? "FAIL" : "PASS", NSPECIALS);
    failures += bad != 0;
}

/* ── Round and Trunc ─────────────────────────────────────────────────────── */

static uint64_t indefinite(int width) {
    return width == 16 ? 0x8000U : width == 32 ? 0x80000000U : 0x8000000000000000ULL;
}

/* x87 integer store of a binary64 value under rounding control rc, as the
 * low `width` bits of the result. */
static uint64_t ref_store(double x, int rc, int width) {
    if (isnan(x)) {
        return indefinite(width);
    }
    double r;
    if (rc == 0) {
        const double f = floor(x);
        const double frac = x - f;
        if (frac > 0.5) {
            r = f + 1;
        } else if (frac < 0.5) {
            r = f;
        } else {
            r = fmod(f, 2.0) == 0 ? f : f + 1;
        }
    } else if (rc == 1) {
        r = floor(x);
    } else if (rc == 2) {
        r = ceil(x);
    } else {
        r = trunc(x);
    }
    const double lim = ldexp(1.0, width - 1);
    if (r < -lim || r >= lim) {
        return indefinite(width);
    }
    const uint64_t mask = width == 64 ? ~0ULL : (1ULL << width) - 1;
    return (uint64_t)(int64_t)r & mask;
}

static const double kRoundInputs[] = {
    0.0,
    -0.0,
    0.5,
    -0.5,
    1.5,
    -1.5,
    2.5,
    -2.5,
    0.49999999999999994,
    -0.49999999999999994,
    0.75,
    -0.75,
    1e-310,
    -1e-310,
    123456.75,
    -123456.25,
    4503599627370495.5,
    -4503599627370495.5,
    9007199254740994.0,
    32767.5,
    -32768.5,
    32768.0,
    -32769.0,
    2147483647.5,
    -2147483648.5,
    2147483648.0,
    9223372036854774784.0,
    9223372036854775808.0,
    -9223372036854775808.0,
    1e19,
    -1e19,
    1e300,
    INFINITY,
    -INFINITY,
    NAN,
};
#define NROUND ((int)(sizeof(kRoundInputs) / sizeof(kRoundInputs[0])))

static void test_round_shapes(void) {
    int bad = 0;
    int cases = 0;
    for (int i = 0; i < NSPECIALS; i++) {
        int64_t d = 0;
        round_i64(&kSpecials[i], &d);
        cases++;
        if (d != kSpecials[i]) {
            printf("FAIL  Round shape fild/jmp/fistp 0x%016llx got=0x%016llx\n",
                   (unsigned long long)kSpecials[i], (unsigned long long)d);
            bad++;
        }
    }
    for (int rc = 0; rc < 4; rc++) {
        for (int i = 0; i < NROUND; i++) {
            const double x = kRoundInputs[i];
            int64_t d64 = 0;
            int32_t d32 = 0;
            int16_t d16 = 0;
            int32_t f32 = 0;
            int16_t f16 = 0;
            set_cw((uint16_t)(CW_DEFAULT | (rc << 10)));
            round_d64(&x, &d64);
            round_d32(&x, &d32);
            round_d16(&x, &d16);
            fist_d32(&x, &f32);
            fist_d16(&x, &f16);
            set_cw(CW_DEFAULT);
            const uint64_t got[5] = {(uint64_t)d64, (uint32_t)d32, (uint16_t)d16, (uint32_t)f32,
                                     (uint16_t)f16};
            const int widths[5] = {64, 32, 16, 32, 16};
            static const char* const names[5] = {"fistp m64", "fistp m32", "fistp m16",
                                                 "fist m32", "fist m16"};
            for (int k = 0; k < 5; k++) {
                const uint64_t want = ref_store(x, rc, widths[k]);
                cases++;
                if (got[k] != want) {
                    printf("FAIL  %s rc=%d x=%.17g got=0x%llx want=0x%llx\n", names[k], rc, x,
                           (unsigned long long)got[k], (unsigned long long)want);
                    bad++;
                }
            }
        }
    }
    printf("%s  Round shapes, every RC and width (%d cases, %d bad)\n", bad ? "FAIL" : "PASS",
           cases, bad);
    failures += bad != 0;
}

static void test_trunc_shapes(void) {
    int bad = 0;
    int cases = 0;
    for (int rc = 0; rc < 4; rc++) {
        for (int i = 0; i < NROUND; i++) {
            const double x = kRoundInputs[i];
            int64_t t = 0;
            int64_t d64 = 0;
            int32_t d32 = 0;
            int16_t d16 = 0;
            uint16_t cw_after = 0;
            /* fisttp and Trunc ignore RC, and Trunc puts the caller's back. */
            set_cw((uint16_t)(CW_DEFAULT | (rc << 10)));
            trunc_d64(&x, &t);
            __asm__ volatile("fnstcw %0" : "=m"(cw_after));
            fisttp_d64(&x, &d64);
            fisttp_d32(&x, &d32);
            fisttp_d16(&x, &d16);
            set_cw(CW_DEFAULT);
            const uint64_t got[4] = {(uint64_t)t, (uint64_t)d64, (uint32_t)d32, (uint16_t)d16};
            const int widths[4] = {64, 64, 32, 16};
            static const char* const names[4] = {"Trunc m64", "fisttp m64", "fisttp m32",
                                                 "fisttp m16"};
            for (int k = 0; k < 4; k++) {
                const uint64_t want = ref_store(x, 3, widths[k]);
                cases++;
                if (got[k] != want) {
                    printf("FAIL  %s rc=%d x=%.17g got=0x%llx want=0x%llx\n", names[k], rc, x,
                           (unsigned long long)got[k], (unsigned long long)want);
                    bad++;
                }
            }
            cases++;
            if (cw_after != (uint16_t)(CW_DEFAULT | (rc << 10))) {
                printf("FAIL  Trunc rc=%d left cw=0x%04x\n", rc, cw_after);
                bad++;
            }
        }
    }
    printf("%s  Trunc and fisttp shapes (%d cases, %d bad)\n", bad ? "FAIL" : "PASS", cases, bad);
    failures += bad != 0;
}

/* ── 32-bit compatibility mode ───────────────────────────────────────────── */

/* Entered by a far jump with eax = source, edx = destination, ecx = count
 * (9..32), esi:edi = the 64-bit return far pointer.  move32_gate is the
 * RTL's 9..32 byte path with its own registers. */
extern char move32_gate[], round32_gate[], roundd32_gate[];
__asm__(
    ".text\n"
    ".p2align 4\n"
    ".intel_syntax noprefix\n"
    ".code32\n"
    ".globl _move32_gate\n"
    "_move32_gate:\n"
    "    sub     ecx, 8\n"
    "    fild    qword ptr [eax+ecx]\n"
    "    fild    qword ptr [eax]\n"
    "    cmp     ecx, 8\n"
    "    jle     Lm32_small16\n"
    "    fild    qword ptr [eax+8]\n"
    "    cmp     ecx, 16\n"
    "    jle     Lm32_small24\n"
    "    fild    qword ptr [eax+16]\n"
    "    fistp   qword ptr [edx+16]\n"
    "Lm32_small24:\n"
    "    fistp   qword ptr [edx+8]\n"
    "Lm32_small16:\n"
    "    fistp   qword ptr [edx]\n"
    "    fistp   qword ptr [edx+ecx]\n"
    "    push    edi\n"
    "    push    esi\n"
    "    retf\n"
    ".globl _round32_gate\n"
    "_round32_gate:\n"
    "    fild    qword ptr [eax]\n"
    "    jmp     Lr32_store\n"
    "Lr32_store:\n"
    "    fistp   qword ptr [edx]\n"
    "    push    edi\n"
    "    push    esi\n"
    "    retf\n"
    ".globl _roundd32_gate\n"
    "_roundd32_gate:\n"
    "    fld     qword ptr [eax]\n"
    "    jmp     Lrd32_store\n"
    "Lrd32_store:\n"
    "    fistp   dword ptr [edx]\n"
    "    push    edi\n"
    "    push    esi\n"
    "    retf\n"
    ".code64\n"
    ".att_syntax prefix\n");

static uint16_t sel_cs32 = 0;
static uint16_t sel_ds32 = 0;
static uint16_t sel_cs64 = 0;

static uint16_t ldt_sel(int index) {
    return (uint16_t)((index << 3) | (1 << 2) | 3);
}

static uint16_t alloc_ldt_seg(int idx, int is_code) {
    ldt_entry_t e;
    memset(&e, 0, sizeof e);
    if (is_code) {
        e.code.limit00 = 0xFFFF;
        e.code.type = DESC_CODE_READ;
        e.code.dpl = 3;
        e.code.present = 1;
        e.code.limit16 = 0xF;
        e.code.opsz = DESC_CODE_32B;
        e.code.granular = DESC_GRAN_PAGE;
    } else {
        e.data.limit00 = 0xFFFF;
        e.data.type = DESC_DATA_WRITE;
        e.data.dpl = 3;
        e.data.present = 1;
        e.data.limit16 = 0xF;
        e.data.stksz = DESC_DATA_32B;
        e.data.granular = DESC_GRAN_PAGE;
    }
    if (i386_set_ldt(idx, &e, 1) < 0) {
        printf("FAIL  i386_set_ldt(idx=%d,%s) failed (errno=%d %s)\n", idx,
               is_code ? "code" : "data", errno, strerror(errno));
        failures++;
        return 0;
    }
    return ldt_sel(idx);
}

struct __attribute__((packed)) farptr32 {
    uint32_t off;
    uint16_t sel;
};

static void call32(void* gate, const void* eax, void* edx, uint32_t ecx) {
    struct farptr32 to32 = {(uint32_t)(uintptr_t)gate, sel_cs32};
    uint64_t in_eax = (uintptr_t)eax, in_edx = (uintptr_t)edx, in_ecx = ecx;
    uint32_t in_ds32 = sel_ds32, in_cs64 = sel_cs64;
    __asm__ volatile(
        "leaq 1f(%%rip), %%rsi   \n"
        "movl %k[incs], %%edi    \n"
        "mov  %%ds, %%r10w       \n"
        "mov  %%es, %%r8w        \n"
        "mov  %%ss, %%r9w        \n"
        "movl %k[inds], %%eax    \n"
        "mov  %%ax, %%ds         \n"
        "mov  %%ax, %%es         \n"
        "mov  %%ax, %%ss         \n"
        "movl %k[ineax], %%eax   \n"
        "movl %k[inedx], %%edx   \n"
        "movl %k[inecx], %%ecx   \n"
        "ljmp *%[fp]             \n"
        "1:                      \n"
        "mov  %%r10w, %%ds       \n"
        "mov  %%r8w, %%es        \n"
        "mov  %%r9w, %%ss        \n"
        :
        : [ineax] "r"(in_eax), [inedx] "r"(in_edx), [inecx] "r"(in_ecx), [inds] "r"(in_ds32),
          [incs] "r"(in_cs64), [fp] "m"(to32)
        : "rax", "rbx", "rcx", "rdx", "rsi", "rdi", "r8", "r9", "r10", "cc", "memory", "st",
          "st(1)", "st(2)", "st(3)");
}

static void test_compat_mode(void) {
    sel_cs32 = alloc_ldt_seg(3, 1);
    sel_ds32 = alloc_ldt_seg(4, 0);
    __asm__ volatile("mov %%cs, %0" : "=r"(sel_cs64));
    if (sel_cs32 == 0 || sel_ds32 == 0) {
        return;
    }

    /* The 9..32 byte path, separate and overlapping buffers. */
    static const int kCounts32[] = {9, 15, 16, 17, 24, 25, 32};
    static const int kShifts32[] = {0, 1, 13, -1, -13};
    static unsigned char buf[160];
    static unsigned char want[160];
    static unsigned char src[64];
    int bad = 0;
    int cases = 0;
    for (unsigned c = 0; c < sizeof(kCounts32) / sizeof(kCounts32[0]); c++) {
        const int n = kCounts32[c];
        for (int p = 0; p < P_COUNT; p++) {
            for (unsigned s = 0; s < sizeof(kShifts32) / sizeof(kShifts32[0]); s++) {
                const int shift = kShifts32[s];
                char what[96];
                cases++;
                memset(buf, 0xAA, sizeof(buf));
                if (shift == 0) {
                    fill(src, n, p, (int)c);
                    memcpy(want, buf, sizeof(want));
                    memcpy(want + GUARD, src, (size_t)n);
                    call32(move32_gate, src, buf + GUARD, (uint32_t)n);
                } else {
                    fill(buf + 64, n, p, (int)c);
                    memcpy(want, buf, sizeof(want));
                    memmove(want + 64 + shift, want + 64, (size_t)n);
                    call32(move32_gate, buf + 64, buf + 64 + shift, (uint32_t)n);
                }
                snprintf(what, sizeof(what), "32-bit Move count=%d %s shift=%d", n,
                         kPatternNames[p], shift);
                bad += report_mismatch(what, buf, want, (int)sizeof(buf));
            }
        }
    }
    printf("%s  32-bit Move 9..32 bytes, separate and overlapping (%d cases, %d bad)\n",
           bad ? "FAIL" : "PASS", cases, bad);
    failures += bad != 0;

    /* fild, jmp, fistp. */
    bad = 0;
    for (int i = 0; i < NSPECIALS; i++) {
        static int64_t s;
        static int64_t d;
        s = kSpecials[i];
        d = 0;
        call32(round32_gate, &s, &d, 0);
        if (d != s) {
            printf("FAIL  32-bit fild, jmp, fistp 0x%016llx got=0x%016llx\n", (unsigned long long)s,
                   (unsigned long long)d);
            bad++;
        }
    }
    printf("%s  32-bit fild, jmp, fistp (%d values)\n", bad ? "FAIL" : "PASS", NSPECIALS);
    failures += bad != 0;

    /* fld m64, jmp, fistp m32 under every rounding mode. */
    bad = 0;
    cases = 0;
    for (int rc = 0; rc < 4; rc++) {
        for (int i = 0; i < NROUND; i++) {
            static double x;
            static int32_t d;
            x = kRoundInputs[i];
            d = 0;
            set_cw((uint16_t)(CW_DEFAULT | (rc << 10)));
            call32(roundd32_gate, &x, &d, 0);
            set_cw(CW_DEFAULT);
            const uint64_t want32 = ref_store(x, rc, 32);
            cases++;
            if ((uint32_t)d != want32) {
                printf("FAIL  32-bit fistp m32 rc=%d x=%.17g got=0x%x want=0x%llx\n", rc, x,
                       (uint32_t)d, (unsigned long long)want32);
                bad++;
            }
        }
    }
    printf("%s  32-bit Round shape, every RC (%d cases, %d bad)\n", bad ? "FAIL" : "PASS", cases,
           bad);
    failures += bad != 0;
}

/* ── Signal storm ────────────────────────────────────────────────────────── */

enum variant { V_PLAIN, V_MODCTX };

static volatile sig_atomic_t g_variant = V_PLAIN;
static volatile sig_atomic_t g_storm_run = 0;
static volatile sig_atomic_t g_signals = 0;
static volatile unsigned long g_iters = 0;
static pthread_t g_main_thread;

static void handler(int sig, siginfo_t* si, void* ctx) {
    (void)sig;
    (void)si;
    g_signals++;
    if (g_variant == V_MODCTX && ctx != NULL) {
        ucontext_t* uc = (ucontext_t*)ctx;
        /* Same content written back.  Bit 1 of rflags reads as 1 on every
         * x86, so setting it changes nothing the guest can observe. */
        uc->uc_mcontext->__ss.__rflags |= 0x2;
        if (uc->uc_mcsize >= sizeof(*uc->uc_mcontext)) {
            _STRUCT_X86_FLOAT_STATE64 fs;
            memcpy(&fs, &uc->uc_mcontext->__fs, sizeof(fs));
            memcpy(&uc->uc_mcontext->__fs, &fs, sizeof(fs));
        }
    }
}

static void* storm_thread(void* arg) {
    (void)arg;
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGUSR1);
    pthread_sigmask(SIG_BLOCK, &set, NULL);
    unsigned long last = g_iters;
    while (g_storm_run) {
        if (g_iters != last) {
            last = g_iters;
            pthread_kill(g_main_thread, SIGUSR1);
        } else {
            __asm__ volatile("pause");
        }
    }
    return NULL;
}

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

/* One iteration: every Move path on path text and int64 edges, the two-block
 * shape and the Round shape.  Returns the number of byte mismatches. */
static unsigned storm_iteration(void) {
    static const int kStormCounts[] = {9, 17, 25, 33, 100};
    static unsigned char src[160];
    static unsigned char dst[160 + 2 * GUARD];
    static unsigned char want[160 + 2 * GUARD];
    static unsigned char buf[320];
    static unsigned char bwant[320];
    unsigned bad = 0;
    for (unsigned c = 0; c < sizeof(kStormCounts) / sizeof(kStormCounts[0]); c++) {
        const int n = kStormCounts[c];
        for (int p = P_PATH; p <= P_SPECIAL; p++) {
            fill(src, n, p, (int)c);
            memset(dst, 0xAA, sizeof(dst));
            memcpy(want, dst, sizeof(want));
            memcpy(want + GUARD, src, (size_t)n);
            delphi_move(src, dst + GUARD, n);
            bad += memcmp(dst, want, sizeof(dst)) != 0;
            /* Backward loop, then forward loop, on one buffer. */
            for (int dir = 0; dir < 2; dir++) {
                const int so = dir ? 64 : 48;
                const int dof = dir ? 48 : 64;
                memset(buf, 0xAA, sizeof(buf));
                fill(buf + so, n, p, (int)c + 1);
                memcpy(bwant, buf, sizeof(bwant));
                memmove(bwant + dof, bwant + so, (size_t)n);
                delphi_move(buf + so, buf + dof, n);
                bad += memcmp(buf, bwant, sizeof(buf)) != 0;
            }
        }
    }
    for (int i = 0; i < NSPECIALS; i++) {
        int64_t d = 0;
        two_block(&kSpecials[i], &d, (i & 1) ? 9 : 1);
        bad += d != kSpecials[i];
        d = 0;
        round_i64(&kSpecials[i], &d);
        bad += d != kSpecials[i];
    }
    return bad;
}

static void run_storm(int variant, double budget_s) {
    g_variant = variant;
    g_signals = 0;
    g_storm_run = 1;
    pthread_t th;
    const int have_thread = pthread_create(&th, NULL, storm_thread, NULL) == 0;
    unsigned long iters = 0;
    unsigned long bad_iters = 0;
    unsigned long mismatches = 0;
    const double deadline = now_s() + budget_s;
    do {
        const unsigned bad = storm_iteration();
        iters++;
        g_iters = iters;
        if (bad != 0) {
            bad_iters++;
            mismatches += bad;
        }
    } while (now_s() < deadline);
    g_storm_run = 0;
    if (have_thread) {
        pthread_join(th, NULL);
    }
    const char* name = variant == V_MODCTX ? "modctx" : "plain";
    if (!have_thread || bad_iters != 0) {
        printf("FAIL  storm variant=%s  bad iterations=%lu of %lu, mismatches=%lu, signals=%d%s\n",
               name, bad_iters, iters, mismatches, (int)g_signals,
               have_thread ? "" : " (no signal thread)");
        failures++;
    } else {
        printf("PASS  storm variant=%s  iters=%lu signals=%d\n", name, iters, (int)g_signals);
    }
}

int main(int argc, char** argv) {
    const double budget = argc > 1 ? atof(argv[1]) : 0.3;
    setvbuf(stdout, NULL, _IOLBF, 0);

    test_move_separate();
    test_move_overlap();
    test_move_control_word();
    test_two_block();
    test_round_shapes();
    test_trunc_shapes();

    g_main_thread = pthread_self();
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = handler;
    sa.sa_flags = SA_SIGINFO | SA_RESTART;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGUSR1, &sa, NULL) != 0) {
        printf("FAIL  sigaction\n");
        return 1;
    }
    run_storm(V_PLAIN, budget);
    run_storm(V_MODCTX, budget);

    /* Last: once an LDT is installed, signal contexts use the larger
     * layout, which the storm handler does not expect. */
    test_compat_mode();

    printf("%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
