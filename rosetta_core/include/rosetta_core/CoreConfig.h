#pragma once

struct RosettaConfig;

extern const RosettaConfig* g_rosetta_config;

void rosetta_set_config(const RosettaConfig* config);

// X87_PRECISE_FAULTS: off unless the config turns it on.  See
// emit_fault_probe in Translator.cpp.
bool precise_faults_enabled();

// X87_PRECISE_FAULTS=2 (with X87_NATIVE_SLOTS): a run ends before each
// instruction after its first that accesses guest memory, so the probe
// covers every access.
bool precise_split_enabled();
