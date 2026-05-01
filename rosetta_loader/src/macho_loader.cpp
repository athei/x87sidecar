#include "macho_loader.hpp"

#include <mach-o/loader.h>
#include <mach/vm_page_size.h>

#include <algorithm>
#include <fstream>
#include <utility>

auto MachoLoader::open(std::filesystem::path const& path) -> bool {
    if (!std::filesystem::exists(path)) {
        return false;
    }

    auto file = std::ifstream(path, std::ios::binary);

    if (!file.is_open()) {
        return false;
    }

    buffer_ = std::vector<uint8_t>(std::istreambuf_iterator<char>(file),
                                   std::istreambuf_iterator<char>());

    return !buffer_.empty();
}

auto MachoLoader::machHeader() const -> mach_header_64* {
    return reinterpret_cast<mach_header_64*>(const_cast<uint8_t*>(buffer_.data()));
}

auto MachoLoader::imageSize() const -> size_t {
    auto* header = machHeader();

    size_t imageSize = 0;

    auto* cmd = reinterpret_cast<load_command*>(header + 1);

    for (auto i = 0; std::cmp_less(i, header->ncmds); i++) {
        if (cmd->cmd == LC_SEGMENT_64) {
            auto* seg = reinterpret_cast<segment_command_64*>(cmd);

            uint64_t segEnd = seg->vmaddr + seg->vmsize;
            imageSize = std::max<uint64_t>(segEnd, imageSize);
        }

        cmd = reinterpret_cast<load_command*>(reinterpret_cast<uint8_t*>(cmd) + cmd->cmdsize);
    }

    imageSize = (imageSize + vm_page_size - 1) & ~(vm_page_size - 1);
    return imageSize;
}

auto MachoLoader::getSection(const char* segment, const char* section) const -> section_64* {
    auto* header = machHeader();

    auto* cmd = reinterpret_cast<load_command*>(header + 1);

    for (auto i = 0; std::cmp_less(i, header->ncmds); i++) {
        if (cmd->cmd == LC_SEGMENT_64) {
            auto* seg = reinterpret_cast<segment_command_64*>(cmd);

            if (strcmp(seg->segname, segment) == 0) {
                auto* sect = reinterpret_cast<section_64*>(seg + 1);

                for (auto j = 0; std::cmp_less(j, seg->nsects); j++) {
                    if (strcmp(sect->sectname, section) == 0) {
                        return sect;
                    }

                    sect++;
                }
            }
        }

        cmd = reinterpret_cast<load_command*>(reinterpret_cast<uint8_t*>(cmd) + cmd->cmdsize);
    }

    return nullptr;
}

auto MachoLoader::getSegment(const char* segment) const -> segment_command_64* {
    auto* header = machHeader();

    auto* cmd = reinterpret_cast<load_command*>(header + 1);

    for (auto i = 0; std::cmp_less(i, header->ncmds); i++) {
        if (cmd->cmd == LC_SEGMENT_64) {
            auto* seg = reinterpret_cast<segment_command_64*>(cmd);

            if (strcmp(seg->segname, segment) == 0) {
                return seg;
            }
        }

        cmd = reinterpret_cast<load_command*>(reinterpret_cast<uint8_t*>(cmd) + cmd->cmdsize);
    }

    return nullptr;
}

auto MachoLoader::forEachSegment(
    const std::function<void(segment_command_64* segm)>& callback) const -> void {
    auto* header = machHeader();

    auto* cmd = reinterpret_cast<load_command*>(header + 1);

    for (auto i = 0; std::cmp_less(i, header->ncmds); i++) {
        if (cmd->cmd == LC_SEGMENT_64) {
            auto* seg = reinterpret_cast<segment_command_64*>(cmd);

            // Skip __PAGEZERO and any other unmapped segments (initprot=NONE, no file backing)
            if (seg->initprot != VM_PROT_NONE) {
                callback(seg);
            }
        }

        cmd = reinterpret_cast<load_command*>(reinterpret_cast<uint8_t*>(cmd) + cmd->cmdsize);
    }
}
