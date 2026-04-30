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
    double sin_c[7];

    // 0x1p23 — Cody-Waite reduction loses precision past this threshold.
    double range_val;
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
