#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// The runtime's opcode mnemonic table, read out of its __cstring section.
// Split from OffsetFinder so the parse can be fed synthetic bytes in tests.
struct OpcodeTable {
    // Host opcode id -> mnemonic. Empty when the table was not found. Trimmed
    // to count when that is known; otherwise it runs on into whatever strings
    // follow the table.
    std::vector<std::string> names;
    // How many opcodes the runtime has, from the bound in its opcode_to_string
    // assert ("index < N"). 0 when not found.
    std::uint32_t count = 0;
};

// `data` is the __cstring section, `size` its length.
auto parseOpcodeTable(const std::uint8_t* data, std::size_t size) -> OpcodeTable;
