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
