// test_fstpt_gs.c -- FSTP m80fp with GS segment override (TLS)
//
// Exercises the translate_fst FSTP m80fp path when the destination uses
// %gs:-relative addressing (seg_override=2).  This is the code path that
// previously triggered GPR exhaustion in compute_operand_address because
// the TLS segment override needs 3 transient GPRs (inner_reg + seg_reg +
// tls_tmp=X29) while translate_fst had already pinned 6 of the 8 pool
// registers.
//
// The fix defers Xbits/Wexp allocation until after compute_operand_address
// returns, freeing 2 GPRs during the critical window.
//
// Strategy: We use a __thread variable and obtain its GS-relative offset
// by comparing its linear address with the GS base (read from %gs:0 on
// macOS, which is pthread_self, i.e. the value at the GS base).  We then
// store via "fstpt %gs:offset" and read back via the __thread variable's
// linear address.
//
// Compile:
//   clang -arch x86_64 -O0 -g -o test_fstpt_gs test_fstpt_gs.c

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// 10-byte f80 result packed into two integers for comparison
typedef struct {
    uint64_t mantissa;
    uint16_t exponent;
} F80;

static F80 as_f80(const unsigned char buf[10]) {
    F80 r;
    memcpy(&r.mantissa, buf, 8);
    memcpy(&r.exponent, buf + 8, 2);
    return r;
}

// TLS destination buffer.  On macOS x86-64, __thread vars live in the TLS
// block whose base is the GS segment base.
static __thread unsigned char tls_f80_buf[16] __attribute__((aligned(16)));

// ---------------------------------------------------------------------------
// Helper: push a double via FLD m64fp, then store via FSTPT using an
// explicit %gs: segment override on the memory operand.
//
// On macOS x86-64, the GS segment base points to the TLS area and %gs:0
// holds pthread_self().  __thread variables sit at negative offsets from
// pthread_self() (i.e., at gs_base + negative_offset, where gs_base +
// pthread_offset = pthread_self).  We compute the offset of our __thread
// buffer relative to the GS base, then use %gs:offset(%reg) in inline asm
// to force the 0x65 GS prefix on the fstpt instruction.
// ---------------------------------------------------------------------------
static F80 store_f64_as_f80_gs(volatile double* src) {
    memset(tls_f80_buf, 0, sizeof(tls_f80_buf));

    // On macOS x86-64, the GS base can be read via the _os_tsd_get_direct
    // pattern, but the simplest portable way to get the GS-relative offset
    // of a __thread var is to use the __emutls or TLV mechanism.
    //
    // Alternative approach: just use a fixed negative offset from %gs:0.
    // pthread_self() is at %gs:0.  We know our __thread var's linear address.
    // gs_base + 0 = pthread_self(), so gs_base = pthread_self() - 0... no,
    // %gs:0 READS the value at (gs_base + 0), which is pthread_self().
    // The gs_base itself is the TLS block base.
    //
    // Actually on macOS x86-64: the GS base IS pthread_self().  The TSD
    // slots are at pthread_self() + small_offsets.  __thread vars are in a
    // separate TLV block allocated via tlv_get_addr, not at fixed GS offsets.
    //
    // So the correct approach for getting a %gs:-prefixed fstpt: use the
    // pthread TSD area directly.  We write to a TSD slot via %gs: and read
    // it back.  But TSD slots are pointer-sized, not 10-byte.
    //
    // Simplest working approach: store via %gs:N where N is a known-safe
    // offset in the pthread TSD area (slots 0-255 are available), then
    // read back using the same %gs: prefix.  We use two consecutive 8-byte
    // slots to store the 10-byte f80 value.

    // Use TSD slots at offset 0x300 (slot 96) and 0x308 (slot 97).
    // These are in the user-available range on macOS.
    // 0x300 = 96 * 8 bytes from GS base.

    __asm__ volatile(
        "fldl (%[src])\n"
        "fstpt %%gs:0x300\n"
        :
        : [src] "r"(src)
        : "memory", "st");

    // Read back the 10 bytes from the same GS-relative location
    uint64_t mant;
    uint16_t exp;
    __asm__ volatile("movq %%gs:0x300, %0" : "=r"(mant));
    __asm__ volatile("movw %%gs:0x308, %0" : "=r"(exp));

    F80 r = {mant, exp};
    return r;
}

// ---------------------------------------------------------------------------
// Test cases -- same expected values as test_fstpt.c
// ---------------------------------------------------------------------------

// +1.0: f64 exp=1023 -> f80 exp=16383=0x3FFF, mantissa=0x8000000000000000
static int test_fstpt_gs_one(void) {
    volatile double src = 1.0;
    F80 r = store_f64_as_f80_gs(&src);
    return (r.mantissa == 0x8000000000000000ULL && r.exponent == 0x3FFF);
}

// -1.0: sign bit set -> exponent word = 0xBFFF
static int test_fstpt_gs_neg(void) {
    volatile double src = -1.0;
    F80 r = store_f64_as_f80_gs(&src);
    return (r.mantissa == 0x8000000000000000ULL && r.exponent == 0xBFFF);
}

// pi: exp=1024 -> f80 exp=16384=0x4000
static int test_fstpt_gs_pi(void) {
    volatile double src = 3.14159265358979323846;
    F80 r = store_f64_as_f80_gs(&src);
    return (r.mantissa == 0xC90FDAA22168C000ULL && r.exponent == 0x4000);
}

// +inf: exponent = 0x7FFF, mantissa = 0x8000000000000000
static int test_fstpt_gs_inf(void) {
    volatile double src = __builtin_inf();
    F80 r = store_f64_as_f80_gs(&src);
    return (r.mantissa == 0x8000000000000000ULL && r.exponent == 0x7FFF);
}

// +0.0: all zero (zero/denorm path)
static int test_fstpt_gs_zero(void) {
    volatile double src = 0.0;
    F80 r = store_f64_as_f80_gs(&src);
    return (r.mantissa == 0x0000000000000000ULL && r.exponent == 0x0000);
}

// ---------------------------------------------------------------------------

typedef struct {
    const char* name;
    int (*fn)(void);
} TestCase;

int main(void) {
    TestCase tests[] = {
        {"fstpt/gs  +1.0             ", test_fstpt_gs_one},
        {"fstpt/gs  -1.0  sign bit   ", test_fstpt_gs_neg},
        {"fstpt/gs  pi    exp+mant   ", test_fstpt_gs_pi},
        {"fstpt/gs  +inf  exp=7FFF   ", test_fstpt_gs_inf},
        {"fstpt/gs  +0.0  zero path  ", test_fstpt_gs_zero},
    };

    int pass = 0, fail = 0;
    int n = (int)(sizeof(tests) / sizeof(tests[0]));
    for (int i = 0; i < n; i++) {
        int ok = tests[i].fn();
        printf("%s  %s\n", tests[i].name, ok ? "PASS" : "FAIL");
        ok ? pass++ : fail++;
    }

    printf("\n%d/%d passed\n", pass, n);
    return fail ? 1 : 0;
}
