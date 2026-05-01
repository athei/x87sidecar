#include "cli_args.hpp"

#include <cstdio>
#include <cstring>
#include <string_view>

#include "rosetta_core/ConfigCli.h"

namespace {

void print_usage(const char* prog) {
    std::printf(
        "usage: %s [flags] <program> [program-args...]\n"
        "\n"
        "Loader flags:\n"
        "  --help                       print this message and exit\n"
        "  --logs                       verbose loader logging to stdout\n"
        "  --force-attach               attach even for x64 PE binaries (default: skip)\n"
        "  --disable-hook               passthrough mode for benchmark baselines: still\n"
        "                               attaches and writes g_disable_aot=1, but skips the\n"
        "                               translate_insn entry patch (no x87 hook).  Apple's\n"
        "                               runtime then translates with stock JIT codegen,\n"
        "                               giving an apples-to-apples baseline against the\n"
        "                               optimised path (both have AOT cache + interpreter\n"
        "                               disabled).\n"
        "\n",
        prog);
    print_translator_flag_help(stdout);
}

}  // namespace

CliArgs parse_cli_args(int argc, char** argv) {
    CliArgs out{};

    int i = 1;
    for (; i < argc; ++i) {
        std::string_view a{argv[i]};
        if (!a.starts_with("--")) {
            break;  // first non-flag — that's the program path
        }

        if (a == "--help") {
            print_usage(argv[0]);
            out.exit_early = true;
            out.exit_code = 0;
            return out;
        }
        if (a == "--logs") {
            out.cfg.loader_logs = 1;
            continue;
        }
        if (a == "--force-attach") {
            out.cfg.loader_force_attach = 1;
            continue;
        }
        if (a == "--disable-hook") {
            out.cfg.loader_disable_hook = 1;
            continue;
        }

        const char* bad = nullptr;
        if (apply_translator_flag(a, out.cfg, bad)) {
            continue;
        }
        if (bad != nullptr) {
            std::fprintf(stderr, "%s: unknown fusion name in %s: %s\n", argv[0], argv[i], bad);
        } else {
            std::fprintf(stderr, "%s: unknown flag: %s (try --help)\n", argv[0], argv[i]);
        }
        out.exit_early = true;
        out.exit_code = 2;
        return out;
    }

    if (i >= argc) {
        std::fprintf(stderr, "%s: missing <program> argument (try --help)\n", argv[0]);
        out.exit_early = true;
        out.exit_code = 2;
        return out;
    }

    out.program_argv_idx = i;
    return out;
}
