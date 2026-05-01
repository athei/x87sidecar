#pragma once

#include "rosetta_core/Config.h"

// Result of parsing the rosettax87 loader's command line.  Filled with the
// flag-derived `cfg`, the index of the program-path argument, and a
// "should we exit early?" flag (for --help / --version / parse errors).
struct CliArgs {
    RosettaConfig cfg;
    int program_argv_idx;  // index into argv where the target program starts
    bool exit_early;       // true → main returns immediately with `exit_code`
    int exit_code;
};

// Parse argv, recognising flags up to the first non-flag argument.  Pop them
// from the conceptual argv (returns the index of the first non-flag).  On
// `--help` prints usage to stdout and sets exit_early=true.
CliArgs parse_cli_args(int argc, char** argv);
