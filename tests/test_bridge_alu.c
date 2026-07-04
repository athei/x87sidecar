/*
 * test_bridge_alu.c — Run bridging v2: flag-writing ALU instructions in the
 * gaps between x87 segments, bridgeable when Rosetta's flag_liveness byte
 * proves their written flags dead (a later full flag-writer in the block
 * kills them before any reader).
 *
 * Scenarios:
 *   1. Dead-flag 32-bit add/sub in one gap, xor/or in another — all flags
 *      killed by a trailing test; region is v2-bridgeable.  Values and FP
 *      results must be exact whether bridging fires or not.
 *   2. Same with 64-bit ALU and an `and`.
 *   3. inc/dec in the gap (folded to add/sub 1 when bridged).
 *   4. Memory forms: alu r,[m] (RM) and alu [m],r (MR).
 *   5. Live flags: a jcc directly consumes the gap add's ZF — the region
 *      must NOT be v2-bridged (flag_liveness != 0); branch and values must
 *      be correct either way.
 *   6. Flags passing THROUGH the gap: cmp before the region, jc after it,
 *      dec in the gap.  dec does not write CF, so the cmp's CF must reach
 *      the jc whether the dec was bridged (NZCV untouched by design) or
 *      the region fell back.
 *
 * Build: clang -arch x86_64 -O0 -o test_bridge_alu test_bridge_alu.c
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

static void check_u64(const char* name, uint64_t got, uint64_t expected) {
    if (got != expected) {
        printf("FAIL  %-55s  got=0x%016llx  expected=0x%016llx\n", name,
               (unsigned long long)got, (unsigned long long)expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

int main(void) {
    const double a[4] = {2.0, 3.0, 5.0, 7.0};

    /* 1. Dead-flag 32-bit ALU gaps: add/sub then xor/or, killed by the
     *    trailing test before the branch. */
    {
        double out;
        uint32_t rc = 100, rd = 50, rs = 0x0f;
        uint32_t guard = 0;
        __asm__ volatile(
            "fldl   (%5)\n\t"
            "fmull  8(%5)\n\t"
            "addl   $3, %1\n\t" /* gap: flags dead (test below) */
            "subl   $2, %2\n\t" /* gap */
            "fldl   16(%5)\n\t"
            "fmull  24(%5)\n\t"
            "xorl   $0xff, %3\n\t" /* gap */
            "orl    $0x100, %3\n\t" /* gap */
            "faddp\n\t"
            "fstpl  %0\n\t"
            "testl  %1, %1\n\t" /* kills every gap writer's flags */
            "jz     1f\n\t"
            "movl   $1, %4\n\t"
            "1:\n\t"
            : "=m"(out), "+r"(rc), "+r"(rd), "+r"(rs), "+m"(guard)
            : "r"(a)
            : "cc", "st", "st(1)", "memory");
        check("v2 dead 32-bit: fp result", out, a[0] * a[1] + a[2] * a[3]);
        check_u32("v2 dead 32-bit: add", rc, 103);
        check_u32("v2 dead 32-bit: sub", rd, 48);
        check_u32("v2 dead 32-bit: xor+or", rs, 0x1f0);
        check_u32("v2 dead 32-bit: branch", guard, 1);
    }

    /* 2. 64-bit ALU and an `and`. */
    {
        double out;
        uint64_t rc = 0x100000001ULL, rd = 0xffffffffffULL;
        __asm__ volatile(
            "fldl   (%3)\n\t"
            "fmull  8(%3)\n\t"
            "addq   $7, %1\n\t" /* gap (64-bit) */
            "andq   $-16, %2\n\t" /* gap (64-bit logical imm) */
            "fldl   16(%3)\n\t"
            "fmull  24(%3)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            "testq  %1, %1\n\t" /* killer */
            "jz     1f\n\t"
            "1:\n\t"
            : "=m"(out), "+r"(rc), "+r"(rd)
            : "r"(a)
            : "cc", "st", "st(1)", "memory");
        check("v2 dead 64-bit: fp result", out, a[0] * a[1] + a[2] * a[3]);
        check_u64("v2 dead 64-bit: add", rc, 0x100000008ULL);
        check_u64("v2 dead 64-bit: and", rd, 0xfffffffff0ULL);
    }

    /* 3. inc/dec in the gap (dead flags — killer test follows). */
    {
        double out;
        uint32_t ri = 41, rj = 43;
        __asm__ volatile(
            "fldl   (%3)\n\t"
            "fmull  8(%3)\n\t"
            "incl   %1\n\t" /* gap */
            "decl   %2\n\t" /* gap */
            "fldl   16(%3)\n\t"
            "fmull  24(%3)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            "testl  %1, %1\n\t" /* killer */
            "jz     1f\n\t"
            "1:\n\t"
            : "=m"(out), "+r"(ri), "+r"(rj)
            : "r"(a)
            : "cc", "st", "st(1)", "memory");
        check("v2 inc/dec: fp result", out, a[0] * a[1] + a[2] * a[3]);
        check_u32("v2 inc/dec: inc", ri, 42);
        check_u32("v2 inc/dec: dec", rj, 42);
    }

    /* 4. Memory forms: alu r,[m] and alu [m],r. */
    {
        double out;
        uint32_t mem[2] = {1000, 2000};
        uint32_t rc = 7;
        __asm__ volatile(
            "fldl   (%3)\n\t"
            "fmull  8(%3)\n\t"
            "addl   (%2), %1\n\t" /* gap: RM — rc += mem[0] */
            "subl   %1, 4(%2)\n\t" /* gap: MR — mem[1] -= rc */
            "fldl   16(%3)\n\t"
            "fmull  24(%3)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            "testl  %1, %1\n\t" /* killer */
            "jz     1f\n\t"
            "1:\n\t"
            : "=m"(out), "+r"(rc)
            : "r"(mem), "r"(a)
            : "cc", "st", "st(1)", "memory");
        check("v2 mem forms: fp result", out, a[0] * a[1] + a[2] * a[3]);
        check_u32("v2 mem forms: rm add", rc, 1007);
        check_u32("v2 mem forms: mr sub", mem[1], 2000 - 1007);
    }

    /* 5. Live flags: jz consumes the gap add's ZF directly — flag_liveness
     *    is nonzero, the region must fall back, and the branch must see the
     *    add's real flags. */
    for (int pick = 0; pick < 2; pick++) {
        uint32_t rc = pick ? 1 : 5; /* add $-1 → 0 (taken) : 4 (not taken) */
        uint32_t taken = 0xdead;
        double out;
        __asm__ volatile(
            "fldl   (%3)\n\t"
            "fmull  8(%3)\n\t"
            "addl   $-1, %2\n\t" /* gap candidate, but flags LIVE (jz below) */
            "fldl   16(%3)\n\t"
            "fmull  24(%3)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            "movl   $1, %1\n\t"
            "jz     1f\n\t" /* consumes the add's ZF across the x87 code */
            "movl   $0, %1\n\t"
            "1:\n\t"
            : "=m"(out), "=m"(taken), "+r"(rc)
            : "r"(a)
            : "cc", "st", "st(1)", "memory");
        check("v2 live flags: fp result", out, a[0] * a[1] + a[2] * a[3]);
        check_u32(pick ? "v2 live flags: jz taken" : "v2 live flags: jz not taken", taken,
                  pick ? 1U : 0U);
        check_u32("v2 live flags: add value", rc, pick ? 0U : 4U);
    }

    /* 6. Flags THROUGH the gap: cmp's CF must survive a dec (which does not
     *    write CF) and reach the jc after the region — whether the dec was
     *    bridged (NZCV untouched) or the region fell back. */
    for (int pick = 0; pick < 2; pick++) {
        const uint32_t lhs = pick ? 1 : 9; /* 1 < 5 taken; 9 < 5 not taken */
        uint32_t rc = 10;
        uint32_t taken = 0xdead;
        double out;
        __asm__ volatile(
            "cmpl   $5, %3\n\t" /* sets CF BEFORE the region */
            "fldl   (%4)\n\t"
            "fmull  8(%4)\n\t"
            "decl   %2\n\t" /* gap: does not write CF */
            "fldl   16(%4)\n\t"
            "fmull  24(%4)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            "movl   $1, %1\n\t"
            "jc     1f\n\t" /* reads the ORIGINAL cmp's CF */
            "movl   $0, %1\n\t"
            "1:\n\t"
            : "=m"(out), "=m"(taken), "+r"(rc)
            : "r"(lhs), "r"(a)
            : "cc", "st", "st(1)", "memory");
        check("v2 cf passthrough: fp result", out, a[0] * a[1] + a[2] * a[3]);
        check_u32(pick ? "v2 cf passthrough: jc taken" : "v2 cf passthrough: jc not taken",
                  taken, pick ? 1U : 0U);
        check_u32("v2 cf passthrough: dec value", rc, 9);
    }

    if (failures == 0) {
        printf("ALL PASS  (0 failures)\n");
    } else {
        printf("SOME FAILURES  (%d failures)\n", failures);
    }
    return failures == 0 ? 0 : 1;
}
