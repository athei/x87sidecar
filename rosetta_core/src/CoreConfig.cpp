#include "rosetta_core/CoreConfig.h"

#include "rosetta_core/Config.h"
#include "rosetta_core/TranslatorHelpers.hpp"

const RosettaConfig* g_rosetta_config = nullptr;

void rosetta_set_config(const RosettaConfig* config) {
    g_rosetta_config = config;
    configure_alloc_fault_injection(config != nullptr ? config->fail_alloc_every : 0);
}

bool precise_faults_enabled() {
    return g_rosetta_config == nullptr || g_rosetta_config->precise_faults != 0;
}
