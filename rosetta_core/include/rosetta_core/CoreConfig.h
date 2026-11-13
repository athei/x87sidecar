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

// Whether the code of the block being translated is on a page the guest can
// write and rewrites.  The sidecar sets it before each request; anything
// else leaves it false.
void set_guest_code_rewritten(bool rewritten);

// X87_SMC_SPLIT: a run ends after each instruction that stores to guest
// memory, and is not bridged, because the guest rewrites the code (or the
// knob is 2).
bool smc_split_enabled();
