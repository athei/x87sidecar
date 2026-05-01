#pragma once

#include <cstdint>

// Absolute address of the parent-process transcendental-IPC trampoline.
//
// At loader install time the stub blob (`stub_asm::buildTranscendentalHelper`)
// is written into libRosettaRuntime's __TEXT trailing pad at some address
// chosen by main.cpp.  That address is stashed here so the sidecar's
// Translator can bake it into JIT-emitted `MOVZ/MOVK x16, $addr; BLR x16`
// sequences for the 10 transcendental opcodes.
//
// Set once during loader install; read repeatedly during JIT emit.  Plain
// uint64_t — single writer (loader) before any sidecar receive thread sees
// translate requests, so no atomics needed.
namespace rosetta_core {

void set_transcendental_helper_addr(uint64_t addr);
uint64_t get_transcendental_helper_addr();

// Read-only constants used by the inline transcendental polynomials emitted
// in the JIT bytes.  Loader writes one instance into libRosettaRuntime's
// trailing pad next to the IPC trampoline; address is exposed via the
// setter below and the JIT emits `MOVZ/MOVK Xconst, addr; LDR Dt,
// [Xconst, #off]` per coefficient.  Sourced from
// ARM-software/optimized-routines (math/aarch64/advsimd/sin.c).
struct alignas(8) TranscendentalConstants {
    // Shared range-reduction constants (sin/cos/sincos/tan use these).
    double inv_pi;     // 1/π                 0x1.45f306dc9c883p-2
    double pi_1;       // π high              0x1.921fb54442d18p+1
    double pi_2;       // π mid               0x1.1a62633145c06p-53
    double pi_3;       // π low               0x1.c1cd129024e09p-106

    // sin polynomial coefficients (Estrin form, c0..c6 in advsimd/sin.c).
    // Reused by fcos / fsincos: cos.c lists identical c0..c6.
    double sin_c[7];

    // 0x1p23 — Cody-Waite reduction loses precision past this threshold.
    double range_val;

    // 0.5 — used by fcos's range-reduction offset (n = round(x*inv_pi + 0.5) - 0.5).
    double half;

    // ── f2xm1 (advsimd/exp2m1.c) ─────────────────────────────────────────
    // Polynomial coefficients for `2^r - 1` over |r| <= 1/(2N), N=128:
    //   poly = r*(log2_hi + log2_lo) + r²·c1 + r³·c2 + ... + r⁷·c6
    double exp2m1_log2_hi;     // 0x1.62e42fefa39efp-1
    double exp2m1_log2_lo;     // 0x1.abc9e3b39803f3p-56
    double exp2m1_c1;          // 0x1.ebfbdff82c58ep-3
    double exp2m1_c2;          // 0x1.c6b08d71f5804p-5
    double exp2m1_c3;          // 0x1.3b2ab6fee7509p-7
    double exp2m1_c4;          // 0x1.5d1d37eb33b15p-10
    double exp2m1_c5;          // 0x1.423f35f371d9ap-13
    double exp2m1_c6;          // 0x1.e7d57ad9a5f93p-5
    // Range-reduction shift trick: z = x + shift, n = z - shift snaps x
    // to the nearest k/N (k integer, N=128).  shift = 0x1.8p52/N = 0x1.8p45.
    double exp2m1_shift;       // 0x1.8p45 ≈ 5.27765e+13
    double exp2m1_rnd2zero;    // -0x1p-8  (sign threshold for sm1 lookup)
    double exp2m1_tablebound;  // 0x1.5bfffffffffffp-2 ≈ 0.34
    // 1.0 — used by f2xm1's main path (scalem1 = scale - 1.0).
    double one;

    // 128-entry table of 2^(j/128) bit patterns (with biased-zero exponent).
    // The actual scale is `bits_to_double(exp_table[u & 127] + (u << 45))`
    // where `u = bits(x + shift)`.  Source: __v_exp_data in
    // optimized-routines' v_exp_data.c.
    uint64_t exp_table[128];

    // 88-entry table of (2^(j/128) - 1) for j near 0, used to avoid
    // catastrophic cancellation when |x| < tablebound.  First 44 entries
    // cover j=0..43 (positive x); remaining 44 cover j=-44..-1
    // (negative x, indexed via offset 24).  Source: scalem1[] in
    // exp2m1.c.
    uint64_t exp_scalem1[88];
};

void set_transcendental_constants_addr(uint64_t addr);
uint64_t get_transcendental_constants_addr();

// Tag values passed in x0 to the trampoline; sidecar dispatches on them.
enum TranscendentalTag : uint32_t {
    kTransF2xm1   = 0,
    kTransFsin    = 1,
    kTransFcos    = 2,
    kTransFsincos = 3,
    kTransFptan   = 4,
    kTransFpatan  = 5,
    kTransFyl2x   = 6,
    kTransFyl2xp1 = 7,
    kTransFprem   = 8,
    kTransFprem1  = 9,
};

}  // namespace rosetta_core
