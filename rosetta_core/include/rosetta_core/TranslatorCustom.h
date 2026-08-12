#pragma once

struct TranslationResult;
struct IRInstr;

// Translators for opcodes stock Rosetta cannot decode at all, which this
// project adds on top of it. They are reached through the decode_opcode hook
// (rosetta_loader/src/stub_asm.cpp), which borrows a same-shape encoding
// Rosetta does decode and relabels it as one of these synthetic opcodes.
//
// Unlike TranslatorX87, these are general-purpose / legacy-mode instructions,
// so they read and write guest GPRs and NZCV rather than the x87 stack.
namespace TranslatorCustom {

// ARPL r/m16, r16 — opcode 0x63, which is MOVSXD in 64-bit mode, so Rosetta's
// tables carry no ARPL and its decoder rejects the encoding outright. The
// decode hook relabels a borrowed ADD r/m32,r32 as kOpcodeName_arpl.
auto translate_arpl(TranslationResult* a1, IRInstr* a2) -> void;

};  // namespace TranslatorCustom
