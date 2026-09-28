#pragma once

#include <cstdint>
#include <string>
#include <vector>

// MacOS 26.5 has shuffled some opcodes around.
// To preserve backwards compatibility with 26.4, we need to be able to translate between the two
// sets of opcodes.

// translates potential Opcode_26_4 opcode to Opcode
auto opcode_host_to_internal(uint16_t opcode) -> uint16_t;

// translates Opcode to potential Opcode_26_4 opcode, if possible. If not possible, returns the
// original opcode.
auto opcode_internal_to_host(uint16_t opcode) -> uint16_t;

// Returned by the mapping functions for an id the other numbering has no entry
// for (a runtime pseudo-op the translator never handles, or an internal opcode
// the installed runtime does not define). Never a valid array index.
constexpr uint16_t kOpcodeUnmapped = 0xFFFF;

// Build the host<->internal maps from the installed runtime's own mnemonic
// table (the names libRosettaRuntime carries, indexed by its opcode id). Once
// set, both mapping functions use it instead of the version-keyed 26.4 table /
// identity guess, so a runtime with a third numbering (e.g. macOS 15 with a
// RosettaUpdateAuto payload: 26.4's ids shifted by one past fxsave/fxrstor and
// 60 AArch64 encoding-class names appended) maps correctly instead of being
// refused. The synthetic ARPL id becomes the first id past the runtime's table.
void opcode_set_host_table(const std::vector<std::string>& hostNames);
bool opcode_host_table_active();
