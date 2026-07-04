/*
 * test_bridge_mov.c — Run bridging v1: mov r,r / mov r,imm / mov r,[m] /
 * mov [m],r joining two x87 segments.
 *
 * Checks BOTH the FP result and the guest register/memory values the
 * bridges produced — a mis-lowered bridge shows up in one or the other.
 * Meaningful under X87_ENABLE_BRIDGE=1 (run_tests.sh bridge phase); in
 * every other phase the same code validates the unbridged paths.
 *
 * Build: clang -arch x86_64 -O0 -o test_bridge_mov test_bridge_mov.c
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
    const double a[4] = {1.5, 2.5, -3.25, 4.0};

    /* mov r,imm + mov r,r joining fld/fmul to fld/fmul/faddp/fstp. */
    {
        double out;
        uint32_t edx_out = 0;
        __asm__ volatile(
            "fldl   (%2)\n\t"
            "fmull  8(%2)\n\t"
            "movl   $0x1234beef, %%ecx\n\t" /* bridge: mov r,imm */
            "movl   %%ecx, %%edx\n\t"       /* bridge: mov r,r   */
            "fldl   16(%2)\n\t"
            "fmull  24(%2)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            "movl   %%edx, %1\n\t" /* trailing (not bridged): read back edx */
            : "=m"(out), "=m"(edx_out)
            : "r"(a)
            : "ecx", "edx", "st", "st(1)", "memory");
        check("bridge mov imm/rr: fp result", out, a[0] * a[1] + a[2] * a[3]);
        check_u32("bridge mov imm/rr: register value", edx_out, 0x1234beefU);
    }

    /* mov r,[m] and mov [m],r inside the gap. */
    {
        double out;
        uint32_t src_mem = 0xcafe0042U;
        uint32_t dst_mem = 0;
        uint32_t esi_out = 0;
        __asm__ volatile(
            "fldl   (%3)\n\t"
            "fmull  8(%3)\n\t"
            "movl   %2, %%esi\n\t" /* bridge: mov r,[m] */
            "movl   %%esi, %1\n\t" /* bridge: mov [m],r */
            "fldl   16(%3)\n\t"
            "fmull  24(%3)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            : "=m"(out), "=m"(dst_mem)
            : "m"(src_mem), "r"(a)
            : "esi", "st", "st(1)", "memory");
        esi_out = dst_mem;
        check("bridge mov ld/st: fp result", out, a[0] * a[1] + a[2] * a[3]);
        check_u32("bridge mov ld/st: memory round-trip", esi_out, 0xcafe0042U);
    }

    if (failures == 0) {
        printf("ALL PASS  (0 failures)\n");
    } else {
        printf("SOME FAILURES  (%d failures)\n", failures);
    }
    return failures == 0 ? 0 : 1;
}
