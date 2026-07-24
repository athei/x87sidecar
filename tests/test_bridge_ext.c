/*
 * test_bridge_ext.c — Run bridging: movzx/movsx/movsxd (register source)
 * and fwait sitting in the gap between two x87 segments.
 *
 * Checks BOTH the FP result and the extended register value the bridge
 * produced — a mis-lowered UBFX/SBFX shows up in one or the other.
 * Meaningful under X87_ENABLE_BRIDGE=1 (run_tests.sh bridge phase); in
 * every other phase the same code validates the unbridged paths.
 *
 * Build: clang -arch x86_64 -O0 -o test_bridge_ext test_bridge_ext.c
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
        printf("FAIL  %-55s  got=0x%016llx  expected=0x%016llx\n", name, (unsigned long long)got,
               (unsigned long long)expected);
        failures++;
    } else {
        printf("PASS  %s\n", name);
    }
}

int main(void) {
    const double a[4] = {1.5, 2.5, -3.25, 4.0};
    const double expected_fp = a[0] * a[1] + a[2] * a[3];

    /* movzx r32, r8lo — source byte with the sign bit set must NOT extend. */
    {
        double out;
        uint32_t edx_out = 0;
        __asm__ volatile(
            "fldl   (%2)\n\t"
            "fmull  8(%2)\n\t"
            "movl   $0xffffff85, %%ecx\n\t" /* bridge: mov r,imm */
            "movzbl %%cl, %%edx\n\t"        /* bridge: movzx r32,r8lo */
            "fldl   16(%2)\n\t"
            "fmull  24(%2)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            "movl   %%edx, %1\n\t"
            : "=m"(out), "=m"(edx_out)
            : "r"(a)
            : "ecx", "edx", "st", "st(1)", "memory");
        check("bridge movzx r8lo: fp result", out, expected_fp);
        check_u32("bridge movzx r8lo: register value", edx_out, 0x85U);
    }

    /* movzx r32, r8hi (AH) — must extract bits [15:8]. */
    {
        double out;
        uint32_t edx_out = 0;
        __asm__ volatile(
            "fldl   (%2)\n\t"
            "fmull  8(%2)\n\t"
            "movl   $0x1234beef, %%eax\n\t" /* bridge: mov r,imm */
            "movzbl %%ah, %%edx\n\t"        /* bridge: movzx r32,r8hi */
            "fldl   16(%2)\n\t"
            "fmull  24(%2)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            "movl   %%edx, %1\n\t"
            : "=m"(out), "=m"(edx_out)
            : "r"(a)
            : "eax", "edx", "st", "st(1)", "memory");
        check("bridge movzx r8hi(AH): fp result", out, expected_fp);
        check_u32("bridge movzx r8hi(AH): register value", edx_out, 0xbeU);
    }

    /* movzx r32, r16 — upper half of a poisoned source must be dropped. */
    {
        double out;
        uint32_t edx_out = 0;
        __asm__ volatile(
            "fldl   (%2)\n\t"
            "fmull  8(%2)\n\t"
            "movl   $0x8000cafe, %%ecx\n\t" /* bridge: mov r,imm */
            "movzwl %%cx, %%edx\n\t"        /* bridge: movzx r32,r16 */
            "fldl   16(%2)\n\t"
            "fmull  24(%2)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            "movl   %%edx, %1\n\t"
            : "=m"(out), "=m"(edx_out)
            : "r"(a)
            : "ecx", "edx", "st", "st(1)", "memory");
        check("bridge movzx r16: fp result", out, expected_fp);
        check_u32("bridge movzx r16: register value", edx_out, 0xcafeU);
    }

    /* movzx r64, r8 — W-form UBFX must still zero bits [63:32]. */
    {
        double out;
        uint64_t rdx_out = 0;
        __asm__ volatile(
            "fldl   (%2)\n\t"
            "fmull  8(%2)\n\t"
            "movq   $-1, %%rdx\n\t"  /* bridge: poison all 64 dst bits */
            "movzbq %%cl, %%rdx\n\t" /* bridge: movzx r64,r8lo */
            "fldl   16(%2)\n\t"
            "fmull  24(%2)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            "movq   %%rdx, %1\n\t"
            : "=m"(out), "=m"(rdx_out)
            : "r"(a), "c"(0xffffff85UL)
            : "rdx", "st", "st(1)", "memory");
        check("bridge movzx r64,r8: fp result", out, expected_fp);
        check_u64("bridge movzx r64,r8: register value", rdx_out, 0x85ULL);
    }

    /* movsx r32, r8 — negative byte must sign-extend to 32 bits. */
    {
        double out;
        uint32_t edx_out = 0;
        __asm__ volatile(
            "fldl   (%2)\n\t"
            "fmull  8(%2)\n\t"
            "movl   $0x00000085, %%ecx\n\t" /* bridge: mov r,imm */
            "movsbl %%cl, %%edx\n\t"        /* bridge: movsx r32,r8lo */
            "fldl   16(%2)\n\t"
            "fmull  24(%2)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            "movl   %%edx, %1\n\t"
            : "=m"(out), "=m"(edx_out)
            : "r"(a)
            : "ecx", "edx", "st", "st(1)", "memory");
        check("bridge movsx r8: fp result", out, expected_fp);
        check_u32("bridge movsx r8: register value", edx_out, 0xffffff85U);
    }

    /* movsx r32, r16 — negative half must sign-extend to 32 bits. */
    {
        double out;
        uint32_t edx_out = 0;
        __asm__ volatile(
            "fldl   (%2)\n\t"
            "fmull  8(%2)\n\t"
            "movl   $0x00008001, %%ecx\n\t" /* bridge: mov r,imm */
            "movswl %%cx, %%edx\n\t"        /* bridge: movsx r32,r16 */
            "fldl   16(%2)\n\t"
            "fmull  24(%2)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            "movl   %%edx, %1\n\t"
            : "=m"(out), "=m"(edx_out)
            : "r"(a)
            : "ecx", "edx", "st", "st(1)", "memory");
        check("bridge movsx r16: fp result", out, expected_fp);
        check_u32("bridge movsx r16: register value", edx_out, 0xffff8001U);
    }

    /* movsxd r64, r32 — negative dword must sign-extend to 64 bits. */
    {
        double out;
        uint64_t rdx_out = 0;
        __asm__ volatile(
            "fldl   (%2)\n\t"
            "fmull  8(%2)\n\t"
            "movl   $0x80000001, %%ecx\n\t" /* bridge: mov r,imm */
            "movslq %%ecx, %%rdx\n\t"       /* bridge: movsxd r64,r32 */
            "fldl   16(%2)\n\t"
            "fmull  24(%2)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            "movq   %%rdx, %1\n\t"
            : "=m"(out), "=m"(rdx_out)
            : "r"(a)
            : "ecx", "rdx", "st", "st(1)", "memory");
        check("bridge movsxd: fp result", out, expected_fp);
        check_u64("bridge movsxd: register value", rdx_out, 0xffffffff80000001ULL);
    }

    /* fwait mid-run — consumed as a nop, the run must stay joined and the
     * value must be exact. */
    {
        double out;
        __asm__ volatile(
            "fldl   (%1)\n\t"
            "fmull  8(%1)\n\t"
            "fwait\n\t" /* bridge: nop */
            "fldl   16(%1)\n\t"
            "fmull  24(%1)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            : "=m"(out)
            : "r"(a)
            : "st", "st(1)", "memory");
        check("bridge fwait: fp result", out, expected_fp);
    }

    if (failures == 0) {
        printf("ALL PASS  (0 failures)\n");
    } else {
        printf("SOME FAILURES  (%d failures)\n", failures);
    }
    return failures == 0 ? 0 : 1;
}
