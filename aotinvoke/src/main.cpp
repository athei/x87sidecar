#include <sys/mman.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <ios>
#include <print>
#include <string_view>
#include <vector>

#include "RosettaAotApi.h"
#include "rosetta_core/Config.h"
#include "rosetta_core/ConfigEnv.h"
#include "rosetta_core/CoreConfig.h"
#include "rosetta_core/CustomTranslationHook.h"

namespace {

void aotinvoke_usage(const char* prog) {
    std::print(
        "usage: {} [flags] <input.bin> <output.bin>\n"
        "\n"
        "Flags:\n"
        "  --help       print this message and exit\n"
        "  --verbose    dump the parsed IR module before translation\n"
        "\n"
        "All other configuration is via environment variables:\n"
        "\n",
        prog);
    print_env_help(stdout);
}

}  // namespace

int main(int argc, char** argv) try {
    bool verbose = false;
    int positional[2];
    int positional_count = 0;

    for (int i = 1; i < argc; ++i) {
        std::string_view a{argv[i]};
        if (a == "--help") {
            aotinvoke_usage(argv[0]);
            return 0;
        }
        if (a == "--verbose") {
            verbose = true;
            continue;
        }
        if (a.starts_with("--")) {
            std::fprintf(stderr, "%s: unknown flag: %s (try --help)\n", argv[0], argv[i]);
            return 2;
        }
        if (positional_count >= 2) {
            std::fprintf(stderr, "%s: too many positional arguments (try --help)\n", argv[0]);
            return 2;
        }
        positional[positional_count++] = i;
    }

    if (positional_count != 2) {
        aotinvoke_usage(argv[0]);
        return 2;
    }

    static RosettaConfig cfg = load_config_from_env();
    rosetta_set_config(&cfg);
    char* input_arg = argv[positional[0]];
    char* output_arg = argv[positional[1]];

    if (!load_rosetta_aot()) {
        std::print("Failed to load libRosettaAot.dylib\n");
        return 1;
    }

    init_custom_translation_hook(g_rosetta_aot.translate_insn_addr,
                                 g_rosetta_aot.transaction_result_size_addr);

    auto version = g_rosetta_aot.version();
    int offset_size = version >= kAotVersion ? g_runtime_routine_offsets.size()
                                             : g_runtime_routine_offsets.size() - 2;

    g_rosetta_aot.register_runtime_routine_offsets(g_runtime_routine_offsets.data(),
                                                   g_runtime_routine_names.data(), offset_size);

    g_rosetta_aot.register_thread_context_offsets(&g_thread_context_offsets);

    const std::filesystem::path blob_path(input_arg);
    const std::filesystem::path out_path(output_arg);

    std::ifstream blob_file(blob_path, std::ios::binary | std::ios::ate);
    if (!blob_file) {
        std::print("Failed to open blob file: {}\n", blob_path.string());
        return 1;
    }

    const auto blob_size = blob_file.tellg();
    if (blob_size <= 0) {
        std::print("Blob file is empty: {}\n", blob_path.string());
        return 1;
    }

    std::vector<std::uint8_t> code(static_cast<size_t>(blob_size));
    blob_file.seekg(0, std::ios::beg);
    blob_file.read(reinterpret_cast<char*>(code.data()), static_cast<std::streamsize>(code.size()));
    if (!blob_file) {
        std::print("Failed to read blob file: {}\n", blob_path.string());
        return 1;
    }

    const size_t code_len = code.size();

    // 2. Map it readable (the disassembler reads directly through the pointer)
    void* blob = mmap(nullptr, 0x1000, PROT_READ | PROT_WRITE, MAP_ANON | MAP_PRIVATE, -1, 0);
    std::memcpy(blob, code.data(), code_len);

    int64_t insts_fileoff_range = (static_cast<int64_t>(code_len) << 32) | 0;

    std::vector<std::uint32_t> inst_targets = {0};
    std::vector<std::uint64_t> data_in_code;

    auto* module_result = g_rosetta_aot.ir_create(
        reinterpret_cast<uintptr_t>(blob),  // absolute mapped base = "file offset 0"
        0,                                  // min_vmaddr   — irrelevant
        0,                                  // max_vmaddr   — irrelevant
        0,                                  // text_vmaddr_range — irrelevant
        0,                                  // data_vmaddr_range — irrelevant
        insts_fileoff_range,
        0,   // a7_null
        -1,  // a8_negative_1
        0,   // stubs_fileoff_range — no stubs
        0,   // stub_size           — no stubs
        inst_targets, data_in_code);

    if (verbose) {
        g_rosetta_aot.module_print(module_result, 1);
    }

    auto* translate_result = g_rosetta_aot.translate(module_result);

    auto translate_data_size = g_rosetta_aot.translator_get_size(translate_result);
    const auto* translate_data = g_rosetta_aot.translator_get_data(translate_result);

    g_rosetta_aot.apply_internal_fixups(translate_result, 0x1000,
                                        const_cast<std::uint8_t*>(translate_data));
    g_rosetta_aot.apply_segmented_runtime_routine_fixups(
        translate_result, const_cast<std::uint8_t*>(translate_data), 0x1000);

    auto out = std::ofstream(out_path, std::ios::binary);
    out.write(reinterpret_cast<const char*>(translate_data), translate_data_size);
    out.close();
    std::print("Written {} bytes -> {}\n", translate_data_size, out_path.string());

    return 0;
} catch (const std::exception& e) {
    std::fprintf(stderr, "aotinvoke: %s\n", e.what());
    return 1;
} catch (...) {
    std::fprintf(stderr, "aotinvoke: unknown exception\n");
    return 1;
}