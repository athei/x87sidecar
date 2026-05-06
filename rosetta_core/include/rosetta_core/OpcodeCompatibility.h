#pragma once

#include <cstdint>

// MacOS 26.5 has shuffled some opcodes around. 
// To preserve backwards compatibility with 26.4, we need to be able to translate between the two sets of opcodes. 

// translates potential Opcode_26_4 opcode to Opcode
auto opcode_host_to_internal(uint16_t opcode) -> uint16_t;

// translates Opcode to potential Opcode_26_4 opcode, if possible. If not possible, returns the original opcode.
auto opcode_internal_to_host(uint16_t opcode) -> uint16_t;
