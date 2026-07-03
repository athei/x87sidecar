/*
 * test_single_op.c — isolated (run==1) fld / fst / fstp memory ops.
 *
 * Exercises the single-op fast path (TranslatorX87Single.cpp): every x87
 * instruction below is fenced by integer instructions INSIDE the same asm
 * block, so X87Cache::lookahead sees a run of exactly 1 and neither the IR
 * (run >= 3), the peephole (>= 2), nor the register cache (>= 2) can fire.
 * This is the ABI-bridge shape (ST0 return / spill at call boundaries).
 *
 * Verifies both the transported VALUES and the fused status+tag write-back:
 * the fast path commits status_word and tag_word as one combined 32-bit RMW,
 * so TOP and the per-slot tag bits after each op are snapshotted via FNSTENV
 * (a fall-through-to-stock op reading the same X87State memory) and compared
 * against architectural expectations.  Values avoid 0.0 so the kValid=00 tag
 * class matches native hardware in the baseline phase.
 *
 * 32-bit protected-mode FPU env layout (28 bytes):
 *   +0 control_word  +4 status_word (TOP at [13:11])  +8 tag_word
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void fnstenv28(uint8_t* env28) {
    __asm__ volatile("fnstenv %0" : "=m"(*env28)::"memory");
}

/* TOP field only: exception/condition flags (PE, C1, ...) differ by design
 * between the JIT (not modeled — generic path included) and native hardware,
 * e.g. inexact f64->f32 narrowing sets PE+C1 natively.  TOP is what the
 * single-op fast path RMWs, so TOP is what we pin. */
static uint16_t env_top(const uint8_t* e) {
    const uint16_t sw = (uint16_t)e[4] | ((uint16_t)e[5] << 8);
    return (sw >> 11) & 7;
}
static uint16_t env_cw(const uint8_t* e) {
    return (uint16_t)e[0] | ((uint16_t)e[1] << 8);
}
static uint16_t env_tw(const uint8_t* e) {
    return (uint16_t)e[8] | ((uint16_t)e[9] << 8);
}

static void check_eq16(const char* name, uint16_t got, uint16_t expected) {
    if (got == expected) {
        printf("PASS  %s  (0x%04x)\n", name, got);
    } else {
        printf("FAIL  %s  got=0x%04x expected=0x%04x\n", name, got, expected);
        failures++;
    }
}

static void check_eq64(const char* name, uint64_t got, uint64_t expected) {
    if (got == expected) {
        printf("PASS  %s  (0x%016llx)\n", name, (unsigned long long)got);
    } else {
        printf("FAIL  %s  got=0x%016llx expected=0x%016llx\n", name, (unsigned long long)got,
               (unsigned long long)expected);
        failures++;
    }
}

static uint64_t as_u64(double d) {
    uint64_t u;
    __builtin_memcpy(&u, &d, 8);
    return u;
}

static uint32_t as_u32(float f) {
    uint32_t u;
    __builtin_memcpy(&u, &f, 4);
    return u;
}

/* FENCE: two integer ops inside the asm string — breaks the x87 run. */
#define FENCE "xorl %%ecx, %%ecx\n\taddl $1, %%ecx\n\t"

/* ── Test 1: single fld m32 — value widen + TOP/tag commit ────────────────── */
static void test_single_fld_m32(void) {
    __asm__ volatile(".byte 0xDB, 0xE3"); /* FNINIT */
    float src = 1.5f;

    __asm__ volatile(FENCE
                     "flds %0\n\t" /* isolated: run == 1 */
                     FENCE
                     :
                     : "m"(src)
                     : "ecx", "memory");

    uint8_t env[28] __attribute__((aligned(16))) = {0};
    fnstenv28(env);
    /* TOP = (0 - 1) & 7 = 7; slot 7 kValid → TW = 0x3FFF; no flags set. */
    check_eq16("fld m32: TOP=7", env_top(env), 7);
    check_eq16("fld m32: CW untouched", env_cw(env), 0x037F);
    check_eq16("fld m32: TW (slot7 valid)", env_tw(env), 0x3FFF);

    double result = 0.0;
    __asm__ volatile(FENCE
                     "fstpl %0\n\t" /* isolated: run == 1 */
                     FENCE
                     : "=m"(result)
                     :
                     : "ecx", "memory");
    check_eq64("fld m32: 1.5f widened", as_u64(result), 0x3FF8000000000000ULL);

    fnstenv28(env);
    /* Back to empty: TOP = 0, all slots kEmpty. */
    check_eq16("fstp m64: TOP=0", env_top(env), 0);
    check_eq16("fstp m64: CW untouched", env_cw(env), 0x037F);
    check_eq16("fstp m64: TW (all empty)", env_tw(env), 0xFFFF);
}

/* ── Test 2: single fld m64 / fstp m64 — bit-exact round trip ─────────────── */
static void test_single_fld_fstp_m64(void) {
    __asm__ volatile(".byte 0xDB, 0xE3");
    double src = 3.14159265358979311600; /* 0x400921FB54442D18 */
    double result = 0.0;

    __asm__ volatile(FENCE
                     "fldl %1\n\t"
                     FENCE
                     "fstpl %0\n\t"
                     FENCE
                     : "=m"(result)
                     : "m"(src)
                     : "ecx", "memory");
    check_eq64("fld/fstp m64: pi bit-exact", as_u64(result), 0x400921FB54442D18ULL);
}

/* ── Test 3: single fst m32 (non-pop) — narrow, state untouched ───────────── */
static void test_single_fst_m32(void) {
    __asm__ volatile(".byte 0xDB, 0xE3");
    double src = 3.14159265358979311600;
    float narrowed = 0.0f;

    __asm__ volatile(FENCE
                     "fldl %0\n\t"
                     FENCE
                     :
                     : "m"(src)
                     : "ecx", "memory");

    __asm__ volatile(FENCE
                     "fsts %0\n\t" /* isolated non-popping store */
                     FENCE
                     : "=m"(narrowed)
                     :
                     : "ecx", "memory");
    check_eq64("fst m32: pi -> f32", (uint64_t)as_u32(narrowed), 0x40490FDBULL);

    uint8_t env[28] __attribute__((aligned(16))) = {0};
    fnstenv28(env);
    /* Non-popping fst leaves TOP and tags unchanged. */
    check_eq16("fst m32: TOP=7 kept", env_top(env), 7);
    check_eq16("fst m32: TW (slot7 valid)", env_tw(env), 0x3FFF);

    double sink;
    __asm__ volatile(FENCE "fstpl %0\n\t" FENCE : "=m"(sink) : : "ecx", "memory");
    check_eq64("fst m32: original kept", as_u64(sink), 0x400921FB54442D18ULL);
}

/* ── Test 4: single fstp at non-empty TOP — dynamic-TOP correctness ───────── */
static void test_single_fstp_dynamic_top(void) {
    __asm__ volatile(".byte 0xDB, 0xE3");
    /* Two adjacent pushes (generic/peephole path): TOP = 6, slots 6+7 valid. */
    __asm__ volatile(
        "fld1\n\t"
        "fld1\n\t" ::
            : "memory");

    uint8_t env[28] __attribute__((aligned(16))) = {0};
    fnstenv28(env);
    check_eq16("dyn: TOP=6", env_top(env), 6);
    check_eq16("dyn: TW (slots 6,7 valid)", env_tw(env), 0x0FFF);

    /* Isolated fstp with TOP=6: value pops, slot 6 -> kEmpty, TOP -> 7. */
    double a = 0.0;
    __asm__ volatile(FENCE "fstpl %0\n\t" FENCE : "=m"(a) : : "ecx", "memory");
    check_eq64("dyn: first pop = 1.0", as_u64(a), 0x3FF0000000000000ULL);

    fnstenv28(env);
    check_eq16("dyn: TOP=7", env_top(env), 7);
    check_eq16("dyn: CW untouched", env_cw(env), 0x037F);
    check_eq16("dyn: TW (slot7 valid)", env_tw(env), 0x3FFF);

    /* Second isolated fstp: back to empty. */
    double b = 0.0;
    __asm__ volatile(FENCE "fstpl %0\n\t" FENCE : "=m"(b) : : "ecx", "memory");
    check_eq64("dyn: second pop = 1.0", as_u64(b), 0x3FF0000000000000ULL);

    fnstenv28(env);
    check_eq16("dyn: TOP=0", env_top(env), 0);
    check_eq16("dyn: TW (all empty)", env_tw(env), 0xFFFF);
}

int main(void) {
    test_single_fld_m32();
    test_single_fld_fstp_m64();
    test_single_fst_m32();
    test_single_fstp_dynamic_top();

    if (failures == 0) {
        printf("\nALL PASS\n");
        return 0;
    }
    printf("\n%d FAILURES\n", failures);
    return 1;
}
