/*
 * test_bridge_flags.c — Run bridging v1: the bridged emission must be
 * NZCV-transparent.  x86 flags set by a cmp BEFORE the bridged region are
 * consumed by a jcc AFTER it; v1 bridges (mov/lea) neither read nor write
 * EFLAGS, so the branch must see the original comparison result.
 *
 * Also verifies fallback correctness: a gap containing a 16-bit mov is NOT
 * v1-eligible, so that region must fall back to plain dispatch and still
 * produce correct values.
 *
 * Build: clang -arch x86_64 -O0 -o test_bridge_flags test_bridge_flags.c
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;
static uint64_t as_u64(double d) {
    uint64_t u;
    memcpy(&u, &d, 8);
    return u;
}

static void check(const char* name, double got, double expected) {
    if (as_u64(got) != as_u64(expected)) {
        printf("FAIL  %-55s  got=%.17g  expected=%.17g\n", name, got, expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

static void check_u32(const char* name, uint32_t got, uint32_t expected) {
    if (got != expected) {
        printf("FAIL  %-55s  got=0x%08x  expected=0x%08x\n", name, got, expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

int main(void) {
    const double a[4] = {2.0, 3.0, 5.0, 7.0};

    /* cmp sets CF/ZF before the run; jbe after the run must honor it.
     * (The cmp/jcc pair sits in the same basic block as the bridged
     * region only up to the branch — Rosetta ends the block there, so the
     * bridged region is the straight-line FP code in between.) */
    for (int pick = 0; pick < 2; pick++) {
        const uint32_t lhs = pick ? 1 : 9;  /* 1 <= 5 taken; 9 <= 5 not taken */
        uint32_t taken = 0xdead;
        double out;
        __asm__ volatile(
            "cmpl   $5, %2\n\t" /* sets flags BEFORE the bridged region */
            "fldl   (%3)\n\t"
            "fmull  8(%3)\n\t"
            "movl   $77, %%ecx\n\t" /* bridge */
            "leal   (%%ecx), %%edx\n\t" /* bridge */
            "fldl   16(%3)\n\t"
            "fmull  24(%3)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            "movl   $1, %1\n\t"
            "jbe    1f\n\t" /* consumes the ORIGINAL cmp flags */
            "movl   $0, %1\n\t"
            "1:\n\t"
            : "=m"(out), "=m"(taken)
            : "r"(lhs), "r"(a)
            : "ecx", "edx", "cc", "st", "st(1)", "memory");
        check("bridge flags: fp result", out, a[0] * a[1] + a[2] * a[3]);
        check_u32(pick ? "bridge flags: jbe taken" : "bridge flags: jbe not taken", taken,
                  pick ? 1U : 0U);
    }

    /* Rejection: a 16-bit mov in the gap is not v1-eligible — the region
     * must fall back (values still exact via the plain paths). */
    {
        double out;
        uint16_t w = 0;
        __asm__ volatile(
            "fldl   (%2)\n\t"
            "fmull  8(%2)\n\t"
            "movw   $0x0042, %%cx\n\t" /* NOT bridgeable (16-bit) */
            "movw   %%cx, %1\n\t"
            "fldl   16(%2)\n\t"
            "fmull  24(%2)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            : "=m"(out), "=m"(w)
            : "r"(a)
            : "ecx", "st", "st(1)", "memory");
        check("bridge reject 16-bit: fp result", out, a[0] * a[1] + a[2] * a[3]);
        check_u32("bridge reject 16-bit: value", w, 0x0042U);
    }

    if (failures == 0) {
        printf("ALL PASS  (0 failures)\n");
    } else {
        printf("SOME FAILURES  (%d failures)\n", failures);
    }
    return failures == 0 ? 0 : 1;
}
