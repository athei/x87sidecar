#include "rosetta_core/IRInstr.h"

#include <cstdint>

#include "rosetta_core/OpcodeCompatibility.h"

auto IRInstr::opcode() const -> uint16_t {
    return opcode_host_to_internal(opcode_);
}
auto IRInstr::set_opcode(uint16_t op) -> void {
    opcode_ = opcode_internal_to_host(op);
}