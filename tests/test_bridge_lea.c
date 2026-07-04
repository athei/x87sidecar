/*
 * test_bridge_lea.c — Run bridging v1: lea (incl. base+index*scale+disp and
 * the 32-bit zero-extension semantics) joining two x87 segments, plus
 * ordering between bridge memory ops and x87 loads/stores.
 *
 * Build: clang -arch x86_64 -O0 -o test_bridge_lea test_bridge_lea.c
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
    const double a[4] = {0.5, 8.0, -2.0, 3.5};

    /* lea with base+index*scale+disp in the gap. */
    {
        double out;
        uint32_t lea_out = 0;
        __asm__ volatile(
            "movl   $100, %%ecx\n\t" /* before the run (not bridged) */
            "fldl   (%2)\n\t"
            "fmull  8(%2)\n\t"
            "leal   7(%%ecx,%%ecx,4), %%edx\n\t" /* bridge: 100*5+7 = 507 */
            "leal   1(%%edx), %%ecx\n\t"         /* bridge: 508 */
            "fldl   16(%2)\n\t"
            "fmull  24(%2)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            "movl   %%ecx, %1\n\t"
            : "=m"(out), "=m"(lea_out)
            : "r"(a)
            : "ecx", "edx", "st", "st(1)", "memory");
        check("bridge lea: fp result", out, a[0] * a[1] + a[2] * a[3]);
        check_u32("bridge lea: address math", lea_out, 508);
    }

    /* Ordering: x87 store -> bridge load of the same slot -> x87 load of a
     * bridge-stored slot.  Program order must hold within the region. */
    {
        double staging = 0.0;
        uint32_t lo_word = 0;
        uint32_t forty_two = 42;
        uint32_t into_mem = 0;
        double out;
        __asm__ volatile(
            "fldl   (%4)\n\t"
            "fmull  8(%4)\n\t"
            "fstl   %1\n\t"        /* x87 store: staging = a0*a1 (no pop) */
            "movl   %1, %%esi\n\t" /* bridge load reads the JUST-stored low word */
            "movl   %%esi, %3\n\t" /* bridge store forwards it */
            "movl   %5, %%edi\n\t" /* bridge load of an unrelated slot */
            "fldl   16(%4)\n\t"
            "fmull  24(%4)\n\t"
            "faddp\n\t"
            "fstpl  %0\n\t"
            "movl   %%esi, %2\n\t"
            : "=m"(out), "=m"(staging), "=m"(lo_word), "=m"(into_mem)
            : "r"(a), "m"(forty_two)
            : "esi", "edi", "st", "st(1)", "memory");
        check("bridge order: fp result", out, a[0] * a[1] + a[2] * a[3]);
        double expected_staging = a[0] * a[1];
        uint32_t expected_lo;
        memcpy(&expected_lo, &expected_staging, 4);
        check_u32("bridge order: load-after-x87-store", lo_word, expected_lo);
        check_u32("bridge order: store forward", into_mem, expected_lo);
    }

    if (failures == 0) {
        printf("ALL PASS  (0 failures)\n");
    } else {
        printf("SOME FAILURES  (%d failures)\n", failures);
    }
    return failures == 0 ? 0 : 1;
}
