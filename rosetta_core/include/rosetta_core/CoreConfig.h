#pragma once

struct RosettaConfig;

extern const RosettaConfig* g_rosetta_config;

void rosetta_set_config(const RosettaConfig* config);

// X87_PRECISE_FAULTS: off unless the config turns it on.  See
// emit_fault_probe in Translator.cpp.
bool precise_faults_enabled();
