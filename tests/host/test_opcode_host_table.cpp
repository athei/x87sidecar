// Host-side test of the name-derived opcode maps (opcode_set_host_table).
//
// Runs natively; no Rosetta involved. It feeds the table three name lists and
// compares the result with what the pinned tables say:
//   - the internal (26.5+) names, where both maps must be the identity;
//   - the 26.4 names, where both maps must equal the pinned 26.4 table;
//   - a list shaped like macOS 15.7.8's runtime, which is the 26.4 list
//     without the four XSAVE-family opcodes.
// The synthetic ARPL id must be the first id past each list.

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "rosetta_core/Opcode.h"
#include "rosetta_core/OpcodeCompatibility.h"
#include "rosetta_core/Opcode_26_4.h"
#include "rosetta_core/RosettaCore.h"

namespace {

int g_failures = 0;

#define CHECK(cond, ...)                 \
    do {                                 \
        if (!(cond)) {                   \
            ++g_failures;                \
            std::printf("      FAIL  "); \
            std::printf(__VA_ARGS__);    \
            std::printf("\n");           \
        }                                \
    } while (0)

// Rosetta's own opcodes: everything in the internal enum except the synthetic
// ARPL appended after them.
constexpr uint16_t kRealOpcodes = kOpcodeName_arpl;

bool isXsaveFamily(uint16_t internal) {
    return internal == kOpcodeName_xgetbv || internal == kOpcodeName_xrstor ||
           internal == kOpcodeName_xsave || internal == kOpcodeName_xsetbv;
}

void checkRangesContiguous(const char* what) {
    static constexpr uint16_t kRanges[][2] = {
        {kOpcodeName_fcmovb, kOpcodeName_fucomip},
        {kOpcodeName_f2xm1, kOpcodeName_fyl2xp1},
    };
    for (const auto& r : kRanges) {
        const uint16_t base = opcode_internal_to_host(r[0]);
        for (uint16_t op = r[0]; op <= r[1]; ++op) {
            CHECK(opcode_internal_to_host(op) == base + (op - r[0]), "%s: %s breaks its x87 range",
                  what, kOpcodeNames[op]);
        }
    }
}

void testIdentity() {
    std::vector<std::string> names;
    for (uint16_t i = 0; i < kRealOpcodes; ++i) {
        names.emplace_back(kOpcodeNames[i]);
    }
    opcode_set_host_table(names);
    CHECK(opcode_host_table_active(), "identity: table not active");
    for (uint16_t i = 0; i < kRealOpcodes; ++i) {
        CHECK(opcode_internal_to_host(i) == i, "identity: %s -> host %u", kOpcodeNames[i],
              opcode_internal_to_host(i));
        CHECK(opcode_host_to_internal(i) == i, "identity: host %u -> %u", i,
              opcode_host_to_internal(i));
    }
    CHECK(opcode_internal_to_host(kOpcodeName_arpl) == kOpcodeName_arpl,
          "identity: arpl at %u, want %u", opcode_internal_to_host(kOpcodeName_arpl),
          kOpcodeName_arpl);
    CHECK(opcode_host_to_internal(kOpcodeName_arpl) == kOpcodeName_arpl,
          "identity: host arpl id does not map back");
    CHECK(opcode_host_to_internal(kOpcodeName_arpl + 1) == kOpcodeUnmapped,
          "identity: an id past arpl is mapped");
    CHECK(opcode_first_unmapped_required() == kOpcodeUnmapped,
          "identity: a required opcode is unmapped");
    checkRangesContiguous("identity");
    opcode_clear_host_table();
}

// The pinned 26.4 table, read through the version-keyed path.
struct Pinned {
    std::vector<uint16_t> hostToInternal;  // by 26.4 host id
    std::vector<uint16_t> internalToHost;  // by internal id
};

Pinned readPinned() {
    opcode_clear_host_table();
    rosetta_core_set_runtime_version(kVersion_26_4);
    Pinned p;
    for (uint16_t h = 0; h < kRealOpcodes; ++h) {
        p.hostToInternal.push_back(opcode_host_to_internal(h));
    }
    for (uint16_t i = 0; i < kRealOpcodes; ++i) {
        p.internalToHost.push_back(opcode_internal_to_host(i));
    }
    // The comparison below is only meaningful if the pinned table is a
    // bijection over Rosetta's own opcodes.
    for (uint16_t h = 0; h < kRealOpcodes; ++h) {
        const uint16_t i = p.hostToInternal[h];
        CHECK(i < kRealOpcodes && p.internalToHost[i] == h,
              "pinned 26.4 table: host %u round-trips to %u", h,
              i < kRealOpcodes ? p.internalToHost[i] : 0xFFFF);
    }
    return p;
}

void test264(const Pinned& p) {
    std::vector<std::string> names;
    for (uint16_t h = 0; h < kRealOpcodes; ++h) {
        names.emplace_back(kOpcodeNames[p.hostToInternal[h]]);
    }
    opcode_set_host_table(names);
    for (uint16_t h = 0; h < kRealOpcodes; ++h) {
        CHECK(opcode_host_to_internal(h) == p.hostToInternal[h], "26.4: host %u -> %u, pinned %u",
              h, opcode_host_to_internal(h), p.hostToInternal[h]);
    }
    for (uint16_t i = 0; i < kRealOpcodes; ++i) {
        CHECK(opcode_internal_to_host(i) == p.internalToHost[i], "26.4: %s -> host %u, pinned %u",
              kOpcodeNames[i], opcode_internal_to_host(i), p.internalToHost[i]);
    }
    CHECK(opcode_internal_to_host(kOpcodeName_arpl) == kOpcode_26_4_arpl,
          "26.4: arpl at %u, want %u", opcode_internal_to_host(kOpcodeName_arpl),
          kOpcode_26_4_arpl);
    CHECK(opcode_host_to_internal(kOpcode_26_4_arpl) == kOpcodeName_arpl,
          "26.4: host arpl id does not map back");
    CHECK(opcode_first_unmapped_required() == kOpcodeUnmapped,
          "26.4: a required opcode is unmapped");
    checkRangesContiguous("26.4");
    opcode_clear_host_table();
}

// macOS 15.7.8: the 26.4 order with xgetbv, xrstor, xsave and xsetbv absent,
// 664 opcodes.
void testSequoia(const Pinned& p) {
    std::vector<std::string> names;
    for (uint16_t h = 0; h < kRealOpcodes; ++h) {
        if (!isXsaveFamily(p.hostToInternal[h])) {
            names.emplace_back(kOpcodeNames[p.hostToInternal[h]]);
        }
    }
    CHECK(names.size() == 664, "15.7.8: list has %zu names, want 664", names.size());
    opcode_set_host_table(names);
    for (uint16_t i = 0; i < kRealOpcodes; ++i) {
        if (isXsaveFamily(i)) {
            CHECK(opcode_internal_to_host(i) == kOpcodeUnmapped, "15.7.8: %s is mapped",
                  kOpcodeNames[i]);
            continue;
        }
        // Each absent opcode with a lower 26.4 id moves this one down by one.
        uint16_t want = p.internalToHost[i];
        for (const uint16_t gone :
             {kOpcodeName_xgetbv, kOpcodeName_xrstor, kOpcodeName_xsave, kOpcodeName_xsetbv}) {
            if (p.internalToHost[gone] < p.internalToHost[i]) {
                --want;
            }
        }
        CHECK(opcode_internal_to_host(i) == want, "15.7.8: %s -> host %u, want %u", kOpcodeNames[i],
              opcode_internal_to_host(i), want);
        CHECK(opcode_host_to_internal(want) == i, "15.7.8: host %u -> %u, want %s", want,
              opcode_host_to_internal(want), kOpcodeNames[i]);
    }
    // Spelled out, since this is the runtime the table exists for.
    CHECK(opcode_internal_to_host(kOpcodeName_xchg) == kOpcode_26_4_xchg, "15.7.8: xchg moved");
    CHECK(opcode_internal_to_host(kOpcodeName_xlat) == kOpcode_26_4_xlat - 1,
          "15.7.8: xlat not one lower");
    CHECK(opcode_internal_to_host(kOpcodeName_xorps) == kOpcode_26_4_xorps - 1,
          "15.7.8: xorps not one lower");
    CHECK(opcode_internal_to_host(kOpcodeName_arpl) == 664, "15.7.8: arpl at %u, want 664",
          opcode_internal_to_host(kOpcodeName_arpl));
    CHECK(opcode_host_to_internal(664) == kOpcodeName_arpl, "15.7.8: host 664 is not arpl");
    CHECK(opcode_host_to_internal(665) == kOpcodeUnmapped, "15.7.8: host 665 is mapped");
    CHECK(opcode_first_unmapped_required() == kOpcodeUnmapped,
          "15.7.8: a required opcode is unmapped");
    checkRangesContiguous("15.7.8");
    opcode_clear_host_table();
}

// A list that lost an opcode the translator reads must be reported, whether the
// opcode is x87 or one that run bridging inspects.
void testMissingRequired() {
    for (const uint16_t gone :
         {static_cast<uint16_t>(kOpcodeName_fld), static_cast<uint16_t>(kOpcodeName_mov)}) {
        std::vector<std::string> names;
        for (uint16_t i = 0; i < kRealOpcodes; ++i) {
            if (i != gone) {
                names.emplace_back(kOpcodeNames[i]);
            }
        }
        opcode_set_host_table(names);
        CHECK(opcode_first_unmapped_required() == gone, "without %s: reported %u",
              kOpcodeNames[gone], opcode_first_unmapped_required());
        opcode_clear_host_table();
    }
}

void testClearRestoresPinned(const Pinned& p) {
    rosetta_core_set_runtime_version(kVersion_26_4);
    CHECK(!opcode_host_table_active(), "clear: table still active");
    CHECK(opcode_first_unmapped_required() == kOpcodeUnmapped,
          "clear: required check fires without a table");
    for (uint16_t i = 0; i < kRealOpcodes; ++i) {
        CHECK(opcode_internal_to_host(i) == p.internalToHost[i], "clear: %s -> host %u, pinned %u",
              kOpcodeNames[i], opcode_internal_to_host(i), p.internalToHost[i]);
    }
}

}  // namespace

int main() {
    testIdentity();
    const Pinned pinned = readPinned();
    test264(pinned);
    testSequoia(pinned);
    testMissingRequired();
    testClearRestoresPinned(pinned);
    std::printf("%d failure(s)\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}
