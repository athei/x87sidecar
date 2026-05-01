/*
 * test_fdisi_feni.c — Tests for FDISI (DB E1) and FENI (DB E0).
 *
 * On 8087 these enabled/disabled FPU interrupts.  On 80287 and later
 * they are unconditionally NOPs — the FPU just ignores them.  Our
 * handler emits no code at all (dispatched to translate_fnop, which
 * goes through x87_begin/x87_end for cache continuity only).
 *
 * Tests:
 *   1. fdisi/feni do not perturb ST(0) or TOP.
 *   2. Surrounding ops still function around fdisi/feni in the same
 *      block (cache stays consistent).
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static uint16_t fnstsw(void) {
    uint16_t sw;
    __asm__ volatile("fnstsw %0" : "=m"(sw));
    return sw;
}

static void check_bitexact_f64(const char* name, double got, double expected) {
    uint64_t g, e;
    memcpy(&g, &got, 8);
    memcpy(&e, &expected, 8);
    if (g == e) {
        printf("PASS  %s  (val=%.17g)\n", name, got);
    } else {
        printf("FAIL  %s  got=%.17g (bits=%016llx) expected=%.17g (bits=%016llx)\n", name, got,
               (unsigned long long)g, expected, (unsigned long long)e);
        failures++;
    }
}

static void check_eq_u16(const char* name, uint16_t got, uint16_t expected) {
    if (got == expected) {
        printf("PASS  %s  (got=0x%04x)\n", name, got);
    } else {
        printf("FAIL  %s  got=0x%04x expected=0x%04x\n", name, got, expected);
        failures++;
    }
}

/* ── Test 1: FDISI is a NOP — TOP and ST(0) unchanged. ────────────────────── */
static void test_fdisi_nop(void) {
    __asm__ volatile("fninit");
    volatile double x = 3.14159;
    __asm__ volatile("fldl %0" : : "m"(x));

    uint16_t sw_before = fnstsw();
    __asm__ volatile(".byte 0xDB, 0xE1"); /* FDISI / FNDISI */
    uint16_t sw_after = fnstsw();

    check_eq_u16("test_fdisi_nop: status_word unchanged", sw_after, sw_before);

    double r;
    __asm__ volatile("fstpl %0" : "=m"(r) : : "st");
    check_bitexact_f64("test_fdisi_nop: ST(0) intact", r, 3.14159);
}

/* ── Test 2: FENI is a NOP — same checks. ─────────────────────────────────── */
static void test_feni_nop(void) {
    __asm__ volatile("fninit");
    volatile double x = -2.71828;
    __asm__ volatile("fldl %0" : : "m"(x));

    uint16_t sw_before = fnstsw();
    __asm__ volatile(".byte 0xDB, 0xE0"); /* FENI / FNENI */
    uint16_t sw_after = fnstsw();

    check_eq_u16("test_feni_nop: status_word unchanged", sw_after, sw_before);

    double r;
    __asm__ volatile("fstpl %0" : "=m"(r) : : "st");
    check_bitexact_f64("test_feni_nop: ST(0) intact", r, -2.71828);
}

/* ── Test 3: surrounding ops still work in the same block. ────────────────── */
static void test_in_run(void) {
    __asm__ volatile("fninit");
    volatile double a = 5.0, b = 7.0;
    /* Build a run: fld a; fdisi; fld b; feni; faddp; fstpl. */
    double r;
    __asm__ volatile(
        "fldl %1\n\t"
        ".byte 0xDB, 0xE1\n\t" /* fdisi */
        "fldl %2\n\t"
        ".byte 0xDB, 0xE0\n\t" /* feni */
        "faddp\n\t"
        "fstpl %0\n\t"
        : "=m"(r)
        : "m"(a), "m"(b)
        : "st");
    check_bitexact_f64("test_in_run: a + b through fdisi/feni", r, 12.0);
}

int main(void) {
    test_fdisi_nop();
    test_feni_nop();
    test_in_run();

    printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
