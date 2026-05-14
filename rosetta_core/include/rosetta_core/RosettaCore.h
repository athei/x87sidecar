#pragma once

#include <cstdint>

void rosetta_core_init(uint64_t runtime_version, uintptr_t translate_insn_addr,
                       uintptr_t transaction_result_size_addr);

// Stores the host Rosetta runtime version for the OpcodeCompatibility layer.
// The loader/sidecar process does not call rosetta_core_init() (that path also
// installs an in-process translate_insn hook, which only aotinvoke wants), so it
// must set the version through this entry point instead.
void rosetta_core_set_runtime_version(uint64_t runtime_version);

uint64_t rosetta_core_runtime_version();
