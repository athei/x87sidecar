#pragma once

#include <cstddef>
#include <cstdint>

struct IRInstr;
struct TranslationResult;

namespace x87trace {

constexpr uint64_t kMagic = UINT64_C(0x0031435254373858);  // X87TRC1
constexpr size_t kCount = 1U << 16;

// Slots have a try-lock, never a spin loop. A stalled writer cannot race a
// later ring wrap; a reservation that finds a busy slot is simply dropped.
struct Record {
    uint64_t locked;
    uint64_t sequence;  // reservation + 1, published with release ordering
    uint64_t context;   // Rosetta X18, identifies the executing thread context
    uint64_t site;      // pc << 32 | instruction index << 1 | exit
    uint64_t nzcv;
    uint64_t fpcr;
    uint64_t native[11];  // CW, SW, tags, eight packed f80 slots, two pad bytes
    uint64_t gpr[15];     // ARM X0..X14, untouched by trace emission
};
static_assert(sizeof(Record) == 256);

struct Control {
    uint64_t next;
    uint64_t frozen;
    uint64_t reserved[30];
};
static_assert(sizeof(Control) == 256);
constexpr size_t kBytes = sizeof(Control) + kCount * sizeof(Record);

struct FileHeader {
    uint64_t magic = kMagic;
    uint64_t version = 1;
    uint64_t hash;
    uint64_t pid;
    uint64_t next;
    uint64_t frozen;
    uint64_t records = kCount;
    uint64_t dropped;
    uint64_t reserved[24]{};
};
static_assert(sizeof(FileHeader) == 256);

// Set once, on the sidecar receive thread, before translating the first
// matching block. The parent VA maps storage also held by the sidecar.
void set_buffer(uint64_t parent, uint64_t hash, bool stop_negative);
bool matches(const IRInstr* ir, size_t count);
void emit_boundary(TranslationResult& tr, uint64_t site, bool end_of_block = false);

}  // namespace x87trace
