#include "rosetta_core/CoreConfig.h"

#include "rosetta_core/Config.h"
#include "rosetta_core/TranslatorHelpers.hpp"

const RosettaConfig* g_rosetta_config = nullptr;

void rosetta_set_config(const RosettaConfig* config) {
    g_rosetta_config = config;
    configure_alloc_fault_injection(config != nullptr ? config->fail_alloc_every : 0);
}

bool precise_faults_enabled() {
    return g_rosetta_config != nullptr && g_rosetta_config->precise_faults != 0;
}

bool precise_split_enabled() {
    return g_rosetta_config != nullptr && g_rosetta_config->precise_faults >= 2 &&
           g_rosetta_config->native_slots != 0;
}

namespace {
bool g_guest_code_rewritten = false;
}

void set_guest_code_rewritten(bool rewritten) {
    g_guest_code_rewritten = rewritten;
}

bool smc_split_enabled() {
    if (g_rosetta_config == nullptr) {
        return false;
    }
    const uint8_t mode = g_rosetta_config->smc_split;
    return mode == 2 || (mode == 1 && g_guest_code_rewritten);
}
