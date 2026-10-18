#pragma once

#include <cstdint>
#include <string>
#include <vector>

// MacOS 26.5 has shuffled some opcodes around.
// To preserve backwards compatibility with 26.4, we need to be able to translate between the two
// sets of opcodes.

// translates a host opcode id to Opcode. Once a host table is set, an id the
// runtime's table has no Opcode for (or one past it, other than the synthetic
// ARPL id) comes back as kOpcodeUnmapped. Without a table it is the identity
// above 26.4 and the pinned 26.4 table at or below it.
auto opcode_host_to_internal(uint16_t opcode) -> uint16_t;

// translates Opcode to a host opcode id. Once a host table is set, an opcode
// the runtime does not define comes back as kOpcodeUnmapped. Without a table it
// is the identity above 26.4; at or below it, an opcode the 26.4 numbering
// lacks comes back as 0.
auto opcode_internal_to_host(uint16_t opcode) -> uint16_t;

// Returned by the mapping functions, once a host table is set, for an id the
// other numbering has no entry for. Never a valid array index.
constexpr uint16_t kOpcodeUnmapped = 0xFFFF;

// Build the host<->internal maps from the installed runtime's own mnemonic
// table: `hostNames[id]` is the runtime's name for opcode `id`, and the list
// holds exactly the runtime's opcodes (the loader trims it to the bound in the
// runtime's opcode_to_string assert). Once set, both mapping functions use it
// instead of the version-keyed 26.4 table / identity guess.
//
// That covers a runtime the version alone misjudges. macOS 15.7.8's reports a
// version on the 26.4 side but has 664 opcodes instead of 668: the four
// XSAVE-family entries (xgetbv, xrstor, xsave, xsetbv) are absent, so every id
// through xchg equals its 26.4 id and xlat, xorpd, xorps sit one lower.
//
// The synthetic ARPL id is the first id past the runtime's opcodes: 668 on
// 26.4 and later, as before, and 664 on that runtime.
void opcode_set_host_table(const std::vector<std::string>& hostNames);
bool opcode_host_table_active();
// Back to the version-keyed mapping. For tests.
void opcode_clear_host_table();

// The first opcode the translator inspects that the host table gives no id
// for, or kOpcodeUnmapped when all of them are mapped (or no table is set).
// These are both x87 ranges and the non-x87 opcodes run bridging reads. A
// runtime missing one of them would have it silently never match.
auto opcode_first_unmapped_required() -> uint16_t;
