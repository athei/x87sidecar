#pragma once

#include <cstdint>

void rosetta_core_init(uint64_t runtime_version,
                       uintptr_t translate_insn_addr,
                       uintptr_t transaction_result_size_addr);

uint64_t rosetta_core_runtime_version();
