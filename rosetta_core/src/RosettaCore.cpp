#include "rosetta_core/RosettaCore.h"

#include <cstdint>

#include "rosetta_core/CustomTranslationHook.h"

static uint64_t g_runtime_version = 0;

void rosetta_core_init(uint64_t runtime_version, uintptr_t translate_insn_addr,
                       uintptr_t transaction_result_size_addr) {
    g_runtime_version = runtime_version;
    init_custom_translation_hook(translate_insn_addr, transaction_result_size_addr);
}

uint64_t rosetta_core_runtime_version() {
    return g_runtime_version;
}
