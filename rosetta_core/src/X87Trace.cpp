#include "rosetta_core/X87Trace.h"

#include "rosetta_core/AssemblerHelpers.hpp"
#include "rosetta_core/ProfileRuntime.h"
#include "rosetta_core/TranslatorX87Helpers.hpp"

namespace x87trace {
namespace {
uint64_t parent_addr;
uint64_t block_hash;
bool freeze_negative;

void publish(AssemblerBuffer& buf, int value, int address) {
    // SWPAL Xvalue, XZR, [Xaddress]. Rosetta's recovery interpreter rejects
    // STLR even though the hardware accepts it; use its LSE atomic family.
    buf.emit(0xF8E08000U | (uint32_t(value) << 16) | (uint32_t(address) << 5) | GPR::XZR);
}
}  // namespace

void set_buffer(uint64_t parent, uint64_t hash, bool stop_negative) {
    parent_addr = parent;
    block_hash = hash;
    freeze_negative = stop_negative;
}

bool matches(const IRInstr* ir, size_t count) {
    return parent_addr != 0 && profile::hash_ir_stream(ir, count) == block_hash;
}

void emit_boundary(TranslationResult& tr, uint64_t site, bool end_of_block) {
    const auto saved_mask = tr.free_gpr_mask;
    tr.free_gpr_mask &= kGprScratchMask;
    const int header = alloc_free_gpr(tr);
    const int record = alloc_free_gpr(tr);
    const int ticket = alloc_free_gpr(tr);
    const int state = alloc_free_gpr(tr);
    const int value = alloc_free_gpr(tr);
    auto& buf = tr.insn_buf;

    // No instruction below writes NZCV or an FP register. All branches
    // go forward, and busy writers are skipped rather than waited for.
    emit_movz_movk_abs64(buf, header, parent_addr);
    emit_ldr_imm(buf, 3, value, header, 1);
    const auto frozen = buf.end;
    emit_cbz(buf, 1, 1, value, 0);
    emit_movn(buf, 1, 2, 0, 1, value);
    emit_ldaddal_x(buf, value, ticket, header);
    emit_and_imm(buf, 1, record, 1, 0, 15, ticket);
    emit_add_sub_shifted_reg(buf, 1, 0, 0, 0, record, 8, header, record);
    emit_add_imm(buf, 1, 0, 0, 0, sizeof(Control), record, record);

    // LDSETAL Xvalue, Xvalue, [Xrecord]: acquire the one-bit slot lock.
    buf.emit(0xF8E03000U | (uint32_t(value) << 16) | (uint32_t(record) << 5) | uint32_t(value));
    const auto busy = buf.end;
    emit_cbz(buf, 1, 1, value, 0);
    emit_str_imm(buf, 3, GPR::XZR, record, 1);
    emit_str_imm(buf, 3, GPR::X18, record, 2);
    emit_movz_movk_abs64(buf, value, site);
    emit_str_imm(buf, 3, value, record, 3);
    emit_mrs_nzcv(buf, value);
    emit_str_imm(buf, 3, value, record, 4);
    buf.emit(0xD53B4400U | uint32_t(value));  // MRS Xvalue, FPCR
    emit_str_imm(buf, 3, value, record, 5);
    emit_x87_base(buf, tr, state);
    for (int i = 0; i < 11; ++i) {
        emit_ldr_imm(buf, 3, value, state, i);
        emit_str_imm(buf, 3, value, record, 6 + i);
    }
    for (int i = 0; i < 15; ++i)
        emit_str_imm(buf, 3, i, record, 17 + i);

    emit_add_imm(buf, 1, 0, 0, 0, 1, ticket, ticket);
    emit_add_imm(buf, 1, 0, 0, 0, 8, record, value);
    publish(buf, ticket, value);
    publish(buf, GPR::XZR, record);

    if (freeze_negative && end_of_block) {
        // ST(0)'s physical slot is TOP, at 6 + 10*TOP. Freeze only the
        // trace buffer, after publishing this record; guest execution
        // and the bad value are left intact for the original failure.
        emit_ldr_imm(buf, 1, value, state, 1);
        emit_bitfield(buf, 0, 2, 0, 11, 13, value, value);
        emit_add_sub_shifted_reg(buf, 1, 0, 0, 0, value, 1, state, state);
        emit_add_sub_shifted_reg(buf, 1, 0, 0, 0, value, 3, state, state);
        emit_ldr_imm(buf, 1, value, state, 7);
        const auto positive = buf.end;
        buf.emit(0x36000000U | (15U << 19) | uint32_t(value));
        emit_add_imm(buf, 1, 0, 0, 0, 8, header, header);
        emit_movn(buf, 1, 2, 0, 1, value);
        publish(buf, value, header);
        buf.data[positive / 4] |= uint32_t((buf.end - positive) / 4) << 5;
    }

    buf.data[frozen / 4] |= uint32_t((buf.end - frozen) / 4) << 5;
    buf.data[busy / 4] |= uint32_t((buf.end - busy) / 4) << 5;
    tr.free_gpr_mask = saved_mask;
}

}  // namespace x87trace
