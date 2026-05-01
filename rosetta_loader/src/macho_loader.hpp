#pragma once

#include <mach-o/loader.h>

#include <filesystem>
#include <functional>
#include <vector>

struct MachoLoader {
    auto open(std::filesystem::path const& path) -> bool;
    auto machHeader() const -> mach_header_64*;
    auto imageSize() const -> size_t;
    auto getSection(const char* segment, const char* section) const -> section_64*;
    auto getSegment(const char* segment) const -> segment_command_64*;
    auto forEachSegment(const std::function<void(segment_command_64* segm)>&) const -> void;

    std::vector<uint8_t> buffer_;
};
