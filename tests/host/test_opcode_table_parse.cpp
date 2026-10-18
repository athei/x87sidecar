// Host-side test of parseOpcodeTable (rosetta_loader/src/opcode_table.cpp).
//
// Runs natively on synthetic __cstring blobs laid out like the runtime's:
// "Opcode.cpp", "opcode_to_string", "index < N", "out-of-bounds opcode index",
// then the mnemonic table and the strings the linker placed after it.

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "opcode_table.hpp"

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

// The first opcodes of every runtime's table; the parse anchors on them.
const std::vector<std::string> kHead = {"aaa", "aad", "aam", "aas", "adc", "add"};

std::vector<std::uint8_t> blob(const std::vector<std::string>& strings) {
    std::vector<std::uint8_t> out;
    for (const auto& s : strings) {
        out.insert(out.end(), s.begin(), s.end());
        out.push_back(0);
    }
    return out;
}

// A table of `count` opcodes: the anchor names, then filler.
std::vector<std::string> opcodes(std::size_t count) {
    std::vector<std::string> names = kHead;
    for (std::size_t i = names.size(); i < count; ++i) {
        names.push_back("op" + std::to_string(i));
    }
    return names;
}

std::vector<std::string> concat(std::initializer_list<std::vector<std::string>> parts) {
    std::vector<std::string> out;
    for (const auto& p : parts) {
        out.insert(out.end(), p.begin(), p.end());
    }
    return out;
}

std::vector<std::string> kPrologue(std::uint32_t n) {
    return {"Opcode.cpp", "opcode_to_string", "index < " + std::to_string(n),
            "out-of-bounds opcode index"};
}

// The AArch64 encoding-class names that follow the table in the real runtime.
const std::vector<std::string> kTrailer = {"addsub_carry", "addsub_ext", "testbranch"};

OpcodeTable parse(const std::vector<std::uint8_t>& bytes) {
    return parseOpcodeTable(bytes.data(), bytes.size());
}

void testRuntimeLayout() {
    for (const std::uint32_t n : {664u, 668u}) {
        const auto bytes = blob(
            concat({{"translate_instruction"}, kPrologue(n), opcodes(n), kTrailer, {"index < 6"}}));
        const OpcodeTable t = parse(bytes);
        CHECK(t.count == n, "layout %u: bound read as %u", n, t.count);
        CHECK(t.names.size() == n, "layout %u: %zu names, want the list cut at the bound", n,
              t.names.size());
        CHECK(!t.names.empty() && t.names.front() == "aaa", "layout %u: list does not start at aaa",
              n);
        CHECK(t.names.size() == n && t.names.back() == "op" + std::to_string(n - 1),
              "layout %u: last name is not the last opcode", n);
    }
}

void testNoBound() {
    // Table present, assert string missing: names run on, count stays 0.
    const auto bytes =
        blob(concat({{"Opcode.cpp", "out-of-bounds opcode index"}, opcodes(20), kTrailer}));
    const OpcodeTable t = parse(bytes);
    CHECK(t.count == 0, "no bound: count %u", t.count);
    CHECK(t.names.size() == 20 + kTrailer.size(), "no bound: %zu names, want the untrimmed list",
          t.names.size());
}

void testForeignBound() {
    // Another "index < N" assert in range, not preceded by opcode_to_string.
    const auto bytes = blob(concat(
        {{"Opcode.cpp", "other_function", "index < 6", "out-of-bounds"}, opcodes(20), kTrailer}));
    const OpcodeTable t = parse(bytes);
    CHECK(t.count == 0, "foreign bound: took %u from an unrelated assert", t.count);

    // The foreign one sits closer to the table than the real one.
    const auto both =
        blob(concat({{"opcode_to_string", "index < 20", "other_function", "index < 6"},
                     opcodes(20),
                     kTrailer}));
    const OpcodeTable u = parse(both);
    CHECK(u.count == 20, "both bounds: read %u, want the opcode_to_string one", u.count);
}

void testBoundTooFarBack() {
    // Only the four strings ahead of the table are searched.
    const auto bytes = blob(
        concat({{"opcode_to_string", "index < 20", "a", "b", "c", "d"}, opcodes(20), kTrailer}));
    const OpcodeTable t = parse(bytes);
    CHECK(t.count == 0, "far bound: read %u from five strings back", t.count);
}

void testMalformedBound() {
    for (const char* bad : {"index < ", "index < 66x", "index < -1"}) {
        const auto bytes =
            blob(concat({{"opcode_to_string", bad, "out-of-bounds"}, opcodes(20), kTrailer}));
        const OpcodeTable t = parse(bytes);
        CHECK(t.count == 0, "malformed \"%s\": read %u", bad, t.count);
    }
}

void testBoundAtSectionStart() {
    // Nothing ahead of the bound: no opcode_to_string, so no count, and no
    // read before the start of the section.
    const auto bytes = blob(concat({{"index < 20"}, opcodes(20), kTrailer}));
    const OpcodeTable t = parse(bytes);
    CHECK(t.count == 0, "bound at section start: read %u", t.count);
}

void testNoTableOrTwo() {
    CHECK(parse(blob({"Opcode.cpp", "aaa", "aad"})).names.empty(), "partial head: names found");
    const auto twice = blob(concat({kPrologue(20), opcodes(20), {"x"}, opcodes(20)}));
    CHECK(parse(twice).names.empty(), "two tables: one was picked");
}

}  // namespace

int main() {
    testRuntimeLayout();
    testNoBound();
    testForeignBound();
    testBoundTooFarBack();
    testMalformedBound();
    testBoundAtSectionStart();
    testNoTableOrTwo();
    if (g_failures != 0) {
        std::printf("%d check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
