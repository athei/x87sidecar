/*
 * test_fstsw_pending_top.c -- FNSTSW reports C3/C2/C0 and TOP while a push,
 * a pop or a tag update from earlier in the same x87 run is still deferred.
 *
 * Build:  clang -arch x86_64 -O0 -g -o test_fstsw_pending_top test_fstsw_pending_top.c
 *
 * Inside a run the translator keeps TOP in a register and writes it back to
 * the status word lazily.  Every path that hands the status word to the guest
 * has to report the live TOP, not the one last written to memory.  The
 * net-zero compare fusions (fld+fcomp+fnstsw, fld+fld+fcompp+fnstsw) skipped
 * that writeback when a push from earlier in the run was still pending, so
 * FNSTSW AX reported TOP as it was before that push.
 *
 * Each case starts from FNINIT (TOP = 0) and runs one x87 run between two
 * CMP instructions, so the deferred state is whatever the run itself left.
 * The cases are shaped to reach each status-word writer with the fusions on
 * (X87_DISABLE_X87_IR=1 sends them through the peephole fusions, the default
 * configuration through the IR's FStsw, X87_DISABLE_ALL_FUSIONS=1 through
 * translate_fstsw).  Expected values are x87 semantics and are checked
 * against stock Rosetta by the native phase.
 *
 * Checked bits: C3 (14), TOP (13:11), C2 (10), C0 (8).
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>

static int failures = 0;

/* C3 | TOP | C2 | C0 */
#define SW_MASK 0x7D00
#define TOP(n) ((uint16_t)((n) << 11))
#define CC_GT 0x0000
#define CC_LT 0x0100
#define CC_EQ 0x4000
#define CC_UN 0x4500

/* Offsets:  0 = 1.75   8 = -3.5   16 = 1.875   24 = 2.5   32 = NaN */
static double in[5] = {1.75, -3.5, 1.875, 2.5, NAN};
/* Offset 0 = 1.875f */
static float in_f[1] = {1.875f};
/* Offset 0 = spill slot for FSTP, offset 8 = FNSTSW m16 destination */
static double scratch[2];

static void check_sw(const char* name, uint16_t got, uint16_t mask, uint16_t expected) {
    if ((got & mask) != expected) {
        printf("FAIL  %-62s  got=0x%04x  expected=0x%04x (mask 0x%04x)\n", name, got & mask,
               expected, mask);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

/*
 * One x87 run, bracketed by FNINIT and a CMP on each side.  %1 = in,
 * %2 = in_f, %3 = scratch.  The run's last FNSTSW AX is the result.
 */
#define SW_CASE(fn, body)                      \
    static uint16_t fn(void) {                 \
        uint16_t ax;                           \
        __asm__ volatile(                      \
            "fninit\n\t"                       \
            "cmpq %%r11, %%r11\n\t" body       \
            "cmpq %%r11, %%r11\n\t"            \
            "fninit\n\t"                       \
            : "=&a"(ax)                        \
            : "r"(in), "r"(in_f), "r"(scratch) \
            : "r11", "cc", "memory");          \
        return ax;                             \
    }

/* ── fld + fcomp/fucomp + fnstsw AX with an earlier push pending ─────────── */

SW_CASE(fld_fcomp_m64_gt,
        "fldl 8(%1)\n\t"
        "fldl 16(%1)\n\t"
        "fcompl (%1)\n\t"
        "fnstsw %%ax\n\t")

SW_CASE(fld_fcomp_m32_lt,
        "fldl (%1)\n\t"
        "fldl 8(%1)\n\t"
        "fcomps (%2)\n\t"
        "fnstsw %%ax\n\t")

SW_CASE(fld_fcomp_m64_eq,
        "fldl (%1)\n\t"
        "fldl 16(%1)\n\t"
        "fcompl 16(%1)\n\t"
        "fnstsw %%ax\n\t")

SW_CASE(fld_fcomp_m64_un,
        "fldl (%1)\n\t"
        "fldl 32(%1)\n\t"
        "fcompl (%1)\n\t"
        "fnstsw %%ax\n\t")

SW_CASE(fld_fucomp_st1_gt,
        "fldl 8(%1)\n\t"
        "fldl (%1)\n\t"
        "fldl 16(%1)\n\t"
        "fucomp %%st(1)\n\t"
        "fnstsw %%ax\n\t")

SW_CASE(fld_st1_fcomp_st1_lt,
        "fldl (%1)\n\t"
        "fldl 24(%1)\n\t"
        "fld %%st(1)\n\t"
        "fcomp %%st(1)\n\t"
        "fnstsw %%ax\n\t")

/* ── fld + fld + fcompp/fucompp [+ fnstsw AX] with an earlier push pending ─ */

SW_CASE(fld_fld_fucompp_lt,
        "fldl (%1)\n\t"
        "fldl 16(%1)\n\t"
        "fldl 8(%1)\n\t"
        "fucompp\n\t"
        "fnstsw %%ax\n\t")

SW_CASE(fld_fld_fcompp_eq,
        "fldl 24(%1)\n\t"
        "fldl (%1)\n\t"
        "fldl (%1)\n\t"
        "fcompp\n\t"
        "fnstsw %%ax\n\t")

SW_CASE(fld_fld_fucompp_un,
        "fldl (%1)\n\t"
        "fldl 32(%1)\n\t"
        "fldl 16(%1)\n\t"
        "fucompp\n\t"
        "fnstsw %%ax\n\t")

/* Three-instruction form, read by a separate FNSTSW later in the run. */
SW_CASE(fld_fld_fucompp_then_fstsw,
        "fldl (%1)\n\t"
        "fldl 16(%1)\n\t"
        "fldl 8(%1)\n\t"
        "fucompp\n\t"
        "fabs\n\t"
        "fnstsw %%ax\n\t")

/* ── fld + fcompp + fnstsw AX with an earlier push pending ───────────────── */

SW_CASE(fld_fcompp_lt,
        "fldl 24(%1)\n\t"
        "fldl (%1)\n\t"
        "fabs\n\t"
        "fldl 8(%1)\n\t"
        "fcompp\n\t"
        "fnstsw %%ax\n\t")

/* ── fcom/fcomp/fucompp + fnstsw AX, fxch + fcom + fnstsw AX ─────────────── */

SW_CASE(fcom_m64_lt,
        "fldl (%1)\n\t"
        "fldl 16(%1)\n\t"
        "fcoml 24(%1)\n\t"
        "fnstsw %%ax\n\t")

SW_CASE(fcomp_st2_gt,
        "fldl (%1)\n\t"
        "fldl 16(%1)\n\t"
        "fldl 24(%1)\n\t"
        "fcomp %%st(2)\n\t"
        "fnstsw %%ax\n\t")

SW_CASE(fucompp_lt,
        "fldl (%1)\n\t"
        "fldl 24(%1)\n\t"
        "fldl 16(%1)\n\t"
        "fabs\n\t"
        "fucompp\n\t"
        "fnstsw %%ax\n\t")

SW_CASE(fxch_fcom_lt,
        "fldl (%1)\n\t"
        "fldl 24(%1)\n\t"
        "fldl 16(%1)\n\t"
        "fxch %%st(2)\n\t"
        "fcoml 24(%1)\n\t"
        "fnstsw %%ax\n\t")

/* ── A pop pending: the stack is filled by an earlier run ────────────────── */

SW_CASE(pop_fld_fcomp_gt,
        "fldl (%1)\n\t"
        "fldl 24(%1)\n\t"
        "cmpq %%r11, %%r11\n\t"
        "fstpl (%3)\n\t"
        "fabs\n\t"
        "fldl 16(%1)\n\t"
        "fcompl (%1)\n\t"
        "fnstsw %%ax\n\t")

SW_CASE(pop_fld_fld_fucompp_lt,
        "fldl (%1)\n\t"
        "fldl 24(%1)\n\t"
        "cmpq %%r11, %%r11\n\t"
        "fstpl (%3)\n\t"
        "fabs\n\t"
        "fldl 16(%1)\n\t"
        "fldl 8(%1)\n\t"
        "fucompp\n\t"
        "fnstsw %%ax\n\t")

SW_CASE(pop_fcom_lt,
        "fldl (%1)\n\t"
        "fldl 24(%1)\n\t"
        "cmpq %%r11, %%r11\n\t"
        "fstpl (%3)\n\t"
        "fcoml 24(%1)\n\t"
        "fnstsw %%ax\n\t")

/* ── Only the tag update pending: TOP already written back by fcom+fnstsw ─ */

SW_CASE(tag_fld_fcomp_gt,
        "fldl (%1)\n\t"
        "fcoml 24(%1)\n\t"
        "fnstsw %%ax\n\t"
        "fldl 16(%1)\n\t"
        "fcompl (%1)\n\t"
        "fnstsw %%ax\n\t")

SW_CASE(tag_fld_fld_fucompp_lt,
        "fldl (%1)\n\t"
        "fcoml 24(%1)\n\t"
        "fnstsw %%ax\n\t"
        "fldl 16(%1)\n\t"
        "fldl 8(%1)\n\t"
        "fucompp\n\t"
        "fnstsw %%ax\n\t")

/* ── FNSTSW m16 after fld + fcomp with a push pending ────────────────────── */

static uint16_t fld_fcomp_fstsw_m16(void) {
    uint16_t sw;
    __asm__ volatile(
        "fninit\n\t"
        "cmpq %%r11, %%r11\n\t"
        "fldl 8(%0)\n\t"
        "fldl 16(%0)\n\t"
        "fcompl (%0)\n\t"
        "fnstsw 8(%1)\n\t"
        "cmpq %%r11, %%r11\n\t"
        "fninit\n\t"
        :
        : "r"(in), "r"(scratch)
        : "r11", "cc", "memory");
    __builtin_memcpy(&sw, (const char*)scratch + 8, 2);
    return sw;
}

int main(void) {
    printf("=== fld + fcomp/fucomp + fnstsw AX, push pending ===\n");
    check_sw("fld m64; fld m64; fcomp m64; fnstsw  (GT, TOP=7)", fld_fcomp_m64_gt(), SW_MASK,
             TOP(7) | CC_GT);
    check_sw("fld m64; fld m64; fcomp m32; fnstsw  (LT, TOP=7)", fld_fcomp_m32_lt(), SW_MASK,
             TOP(7) | CC_LT);
    check_sw("fld m64; fld m64; fcomp m64; fnstsw  (EQ, TOP=7)", fld_fcomp_m64_eq(), SW_MASK,
             TOP(7) | CC_EQ);
    check_sw("fld m64; fld NaN; fcomp m64; fnstsw  (UN, TOP=7)", fld_fcomp_m64_un(), SW_MASK,
             TOP(7) | CC_UN);
    check_sw("fld x3; fucomp st(1); fnstsw  (GT, TOP=6)", fld_fucomp_st1_gt(), SW_MASK,
             TOP(6) | CC_GT);
    check_sw("fld x2; fld st(1); fcomp st(1); fnstsw  (LT, TOP=6)", fld_st1_fcomp_st1_lt(), SW_MASK,
             TOP(6) | CC_LT);

    printf("\n=== fld + fld + fcompp/fucompp + fnstsw AX, push pending ===\n");
    check_sw("fld x3; fucompp; fnstsw  (LT, TOP=7)", fld_fld_fucompp_lt(), SW_MASK, TOP(7) | CC_LT);
    check_sw("fld x3; fcompp; fnstsw  (EQ, TOP=7)", fld_fld_fcompp_eq(), SW_MASK, TOP(7) | CC_EQ);
    check_sw("fld x3 (NaN); fucompp; fnstsw  (UN, TOP=7)", fld_fld_fucompp_un(), SW_MASK,
             TOP(7) | CC_UN);
    /* FABS leaves C0/C2/C3 undefined, so only TOP is checked here. */
    check_sw("fld x3; fucompp; fabs; fnstsw  (TOP=7)", fld_fld_fucompp_then_fstsw(), TOP(7),
             TOP(7));

    printf("\n=== fld + fcompp + fnstsw AX, push pending ===\n");
    check_sw("fld x2; fabs; fld; fcompp; fnstsw  (LT, TOP=7)", fld_fcompp_lt(), SW_MASK,
             TOP(7) | CC_LT);

    printf("\n=== fcom forms + fnstsw AX, push pending ===\n");
    check_sw("fld x2; fcom m64; fnstsw  (LT, TOP=6)", fcom_m64_lt(), SW_MASK, TOP(6) | CC_LT);
    check_sw("fld x3; fcomp st(2); fnstsw  (GT, TOP=6)", fcomp_st2_gt(), SW_MASK, TOP(6) | CC_GT);
    check_sw("fld x3; fabs; fucompp; fnstsw  (LT, TOP=7)", fucompp_lt(), SW_MASK, TOP(7) | CC_LT);
    check_sw("fld x3; fxch st(2); fcom m64; fnstsw  (LT, TOP=5)", fxch_fcom_lt(), SW_MASK,
             TOP(5) | CC_LT);

    printf("\n=== pop pending ===\n");
    check_sw("fstp; fabs; fld; fcomp m64; fnstsw  (GT, TOP=7)", pop_fld_fcomp_gt(), SW_MASK,
             TOP(7) | CC_GT);
    check_sw("fstp; fabs; fld x2; fucompp; fnstsw  (LT, TOP=7)", pop_fld_fld_fucompp_lt(), SW_MASK,
             TOP(7) | CC_LT);
    check_sw("fstp; fcom m64; fnstsw  (LT, TOP=7)", pop_fcom_lt(), SW_MASK, TOP(7) | CC_LT);

    printf("\n=== tag update pending ===\n");
    check_sw("fld; fcom; fnstsw; fld; fcomp m64; fnstsw  (GT, TOP=7)", tag_fld_fcomp_gt(), SW_MASK,
             TOP(7) | CC_GT);
    check_sw("fld; fcom; fnstsw; fld x2; fucompp; fnstsw  (LT, TOP=7)", tag_fld_fld_fucompp_lt(),
             SW_MASK, TOP(7) | CC_LT);

    printf("\n=== fnstsw m16 ===\n");
    check_sw("fld x2; fcomp m64; fnstsw m16  (GT, TOP=7)", fld_fcomp_fstsw_m16(), SW_MASK,
             TOP(7) | CC_GT);

    printf("\n%s: %d failure(s)\n", failures ? "FAILED" : "ALL PASSED", failures);
    return failures ? 1 : 0;
}
