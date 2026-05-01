#pragma once

#include <cstdio>

#include "rosetta_core/Config.h"

// Environment-variable configuration shared by `rosettax87` and `aotinvoke`.
//
// `load_config_from_env` reads every `X87_*` variable in one place and
// returns a fully-populated `RosettaConfig`.  Both binaries call this
// once at startup, before `rosetta_set_config(&cfg)`; no further
// `getenv` should happen anywhere in the project.
//
// `print_env_help` writes a usage block listing every recognised
// `X87_*` variable to `out` (typically stdout).  Both binaries' --help
// implementations call it after their own preamble.
RosettaConfig load_config_from_env();
void print_env_help(std::FILE* out);
