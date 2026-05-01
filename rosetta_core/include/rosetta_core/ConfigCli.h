#pragma once

#include <cstdio>
#include <string_view>

#include "rosetta_core/Config.h"

// Translator-knob CLI flags shared by both `rosettax87` and `aotinvoke`.
//
// `apply_translator_flag` recognises a single flag (with leading `--`) and
// updates `cfg` accordingly.  Returns true if the flag was consumed,
// false if it's not a known translator knob (caller decides what to do
// with unknown flags — error out, or treat as a binary-specific flag).
//
// `print_translator_flag_help` writes a usage block listing every
// translator flag to `out` (typically stdout).  Both binaries' --help
// implementations should call it after their own preamble.
bool apply_translator_flag(std::string_view flag, RosettaConfig& cfg, const char*& bad_value);
void print_translator_flag_help(std::FILE* out);
