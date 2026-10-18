#include "opcode_table.hpp"

#include <algorithm>
#include <charconv>
#include <cstring>
#include <functional>
#include <string_view>
#include <system_error>

// The opcode mnemonic table, in host enum order: what the runtime's own module
// printer uses. The strings that follow it in __cstring belong to other tables
// (the AArch64 encoding-class names come next), so the list is cut at the
// runtime's own count: opcode_to_string asserts "index < N", and that string
// sits just ahead of the table, after "Opcode.cpp" and "opcode_to_string" and
// before "out-of-bounds opcode index".
auto parseOpcodeTable(const std::uint8_t* data, std::size_t size) -> OpcodeTable {
    OpcodeTable table;
    static const char kTableHeadChars[] = "\0aaa\0aad\0aam\0aas\0adc\0add\0";
    const auto* kTableHead = reinterpret_cast<const std::uint8_t*>(kTableHeadChars);
    const auto* begin = data;
    const auto* end = data + size;
    const std::boyer_moore_searcher searcher(kTableHead, kTableHead + sizeof(kTableHeadChars) - 1);
    const auto* head = std::search(begin, end, searcher);
    if (head == end || std::search(head + 1, end, searcher) != end) {
        return table;  // absent, or not unique
    }

    std::size_t p = static_cast<std::size_t>(head - begin) + 1;
    while (p < size && table.names.size() < 1024) {
        const char* s = reinterpret_cast<const char*>(data + p);
        const std::size_t n = strnlen(s, size - p);
        if (n == 0) {
            break;
        }
        table.names.emplace_back(s, n);
        p += n + 1;
    }

    // The string ending at `strEnd` (the offset of its NUL), or an empty view
    // at offset 0 when there is none.
    const auto stringBefore = [&](std::size_t strEnd) -> std::string_view {
        std::size_t strBegin = strEnd;
        while (strBegin > 0 && data[strBegin - 1] != 0) {
            --strBegin;
        }
        return {reinterpret_cast<const char*>(data + strBegin), strEnd - strBegin};
    };

    // Walk back over the few strings ahead of the table for the bound. The
    // runtime has other "index < " asserts, so only the one that follows
    // "opcode_to_string" counts.
    static constexpr std::string_view kBoundPrefix = "index < ";
    std::size_t strEnd = static_cast<std::size_t>(head - begin);
    for (int back = 0; back < 4 && strEnd > 0; ++back) {
        const std::string_view prev = stringBefore(strEnd);
        const std::size_t strBegin =
            static_cast<std::size_t>(reinterpret_cast<const std::uint8_t*>(prev.data()) - data);
        if (prev.starts_with(kBoundPrefix) && strBegin > 0 &&
            stringBefore(strBegin - 1) == "opcode_to_string") {
            std::uint32_t count = 0;
            const char* first = prev.data() + kBoundPrefix.size();
            const char* last = prev.data() + prev.size();
            const auto [ptr, ec] = std::from_chars(first, last, count);
            if (ec == std::errc() && ptr == last && first != last) {
                table.count = count;
            }
            break;
        }
        if (strBegin == 0) {
            break;
        }
        strEnd = strBegin - 1;
    }
    if (table.count != 0 && table.names.size() > table.count) {
        table.names.resize(table.count);
    }
    return table;
}
