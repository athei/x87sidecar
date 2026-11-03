// Host-side test of findArmTreeRoot (rosetta_loader/src/fragment_root.cpp).
//
// Runs natively on synthetic images laid out like the runtime's: __text, then
// __cstring holding the lookup's panic string, then the root global in __DATA
// past the end of the file. The lookup materialises the root and then the
// string, either with adrp/add pairs (macOS 26 and 27) or with adr/nop, where
// the linker relaxed the pairs (macOS 15).

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "fragment_root.hpp"

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

constexpr std::uint64_t kTextBegin = 0x1000;
constexpr std::uint64_t kTextEnd = 0x3000;
constexpr std::uint64_t kPanic = 0x3040;  // in __cstring, right after __text
constexpr std::uint64_t kRoot = 0x5a08;   // a __bss global, past the file
constexpr std::uint32_t kNop = 0xD503201F;

auto pcRel(std::uint32_t op, std::uint32_t rd, std::int64_t imm) -> std::uint32_t {
    const auto bits = static_cast<std::uint32_t>(imm) & 0x1FFFFF;
    return op | ((bits & 0x3) << 29) | ((bits >> 2) << 5) | rd;
}
auto adr(std::uint32_t rd, std::uint64_t pc, std::uint64_t target) -> std::uint32_t {
    return pcRel(0x10000000, rd, static_cast<std::int64_t>(target - pc));
}
auto adrp(std::uint32_t rd, std::uint64_t pc, std::uint64_t target) -> std::uint32_t {
    return pcRel(0x90000000, rd,
                 static_cast<std::int64_t>(target >> 12) - static_cast<std::int64_t>(pc >> 12));
}
auto add(std::uint32_t rd, std::uint32_t rn, std::uint32_t imm12) -> std::uint32_t {
    return 0x91000000 | (imm12 << 10) | (rn << 5) | rd;
}

// An image with `code` at the start of __text and the panic string in place.
struct Image {
    std::vector<std::uint8_t> bytes = std::vector<std::uint8_t>(0x3100, 0);
    std::uint64_t pc = kTextBegin + 0x200;

    void emit(std::uint32_t insn) {
        std::memcpy(bytes.data() + pc, &insn, sizeof(insn));
        pc += 4;
    }
    Image() {
        const char panic[] = "cannot locate code fragment for arm address";
        std::memcpy(bytes.data() + kPanic, panic, sizeof(panic));
    }
    [[nodiscard]] auto root() const -> std::uint64_t {
        return findArmTreeRoot(bytes.data(), bytes.size(), kTextBegin, kTextEnd, kPanic);
    }
};

// The body between the root and the string: lock the root's neighbour, load
// the root, then the search, none of which materialises an address.
void searchBody(Image& im, std::uint32_t rootReg) {
    im.emit(add(0, rootReg, 8));  // add x0, xR, #0x8 (the lock)
    im.emit(0xF9400001 | (rootReg << 5));  // ldr x1, [xR]
    for (int i = 0; i < 12; i++) {
        im.emit(0xF9402C28);  // ldr x8, [x1, #0x58]
    }
}

void testAdrpAdd() {
    Image im;
    const std::uint64_t at = im.pc;
    im.emit(adrp(24, at, kRoot));
    im.emit(add(24, 24, kRoot & 0xFFF));
    searchBody(im, 24);
    const std::uint64_t ref = im.pc;
    im.emit(adrp(0, ref, kPanic));
    im.emit(add(0, 0, kPanic & 0xFFF));
    CHECK(im.root() == kRoot, "adrp/add: root 0x%llx, want 0x%llx",
          static_cast<unsigned long long>(im.root()), static_cast<unsigned long long>(kRoot));
}

void testAdrNop() {
    Image im;
    im.emit(adr(24, im.pc, kRoot));
    im.emit(kNop);
    searchBody(im, 24);
    im.emit(adr(0, im.pc, kPanic));
    im.emit(kNop);
    CHECK(im.root() == kRoot, "adr/nop: root 0x%llx, want 0x%llx",
          static_cast<unsigned long long>(im.root()), static_cast<unsigned long long>(kRoot));
}

void testMixed() {
    // A relaxed root with an unrelaxed string reference, and the reverse.
    Image a;
    a.emit(adr(24, a.pc, kRoot));
    a.emit(kNop);
    searchBody(a, 24);
    a.emit(adrp(0, a.pc, kPanic));
    a.emit(add(0, 0, kPanic & 0xFFF));
    CHECK(a.root() == kRoot, "adr root, adrp/add string: root 0x%llx",
          static_cast<unsigned long long>(a.root()));

    Image b;
    b.emit(adrp(24, b.pc, kRoot));
    b.emit(add(24, 24, kRoot & 0xFFF));
    searchBody(b, 24);
    b.emit(adr(0, b.pc, kPanic));
    b.emit(kNop);
    CHECK(b.root() == kRoot, "adrp/add root, adr string: root 0x%llx",
          static_cast<unsigned long long>(b.root()));
}

void testNegativeImmediate() {
    // adr reaches backwards too: a root placed before the code is not a
    // global past __text, so it must be refused rather than misdecoded.
    Image im;
    im.pc = kTextBegin + 0x1000;
    im.emit(adr(24, im.pc, kTextBegin + 0x10));
    im.emit(kNop);
    searchBody(im, 24);
    im.emit(adr(0, im.pc, kPanic));
    im.emit(kNop);
    CHECK(im.root() == 0, "adr into __text: took 0x%llx",
          static_cast<unsigned long long>(im.root()));
}

void testLocalAdrBetween() {
    // An adr into __text, such as a jump table, between the root and the
    // string is no global: the walk passes over it and still finds the root.
    Image a;
    a.emit(adrp(24, a.pc, kRoot));
    a.emit(add(24, 24, kRoot & 0xFFF));
    searchBody(a, 24);
    a.emit(adr(16, a.pc, kTextBegin + 0x40));
    a.emit(kNop);
    a.emit(adr(0, a.pc, kPanic));
    a.emit(kNop);
    CHECK(a.root() == kRoot, "adrp/add root, local adr x16 after it: root 0x%llx",
          static_cast<unsigned long long>(a.root()));

    Image b;
    b.emit(adr(24, b.pc, kRoot));
    b.emit(kNop);
    searchBody(b, 24);
    b.emit(adr(16, b.pc, kTextBegin + 0x40));
    b.emit(kNop);
    b.emit(adr(0, b.pc, kPanic));
    b.emit(kNop);
    CHECK(b.root() == kRoot, "adr root, local adr x16 after it: root 0x%llx",
          static_cast<unsigned long long>(b.root()));
}

void testOutOfRange() {
    // A target more than 4 MB past __text is no global of this image. adr
    // reaches only 1 MB, so the far target uses adrp/add.
    Image im;
    const std::uint64_t far = kTextEnd + 0x400000 + 0x10;
    im.emit(adrp(24, im.pc, far));
    im.emit(add(24, 24, far & 0xFFF));
    searchBody(im, 24);
    im.emit(adr(0, im.pc, kPanic));
    im.emit(kNop);
    CHECK(im.root() == 0, "root 4 MB past __text: took 0x%llx",
          static_cast<unsigned long long>(im.root()));
}

void testX0IsNotTheRoot() {
    // Only x0 is ever materialised before the string: that is an argument,
    // not the root.
    Image im;
    im.emit(adr(0, im.pc, kRoot));
    im.emit(kNop);
    searchBody(im, 0);
    im.emit(adr(0, im.pc, kPanic));
    im.emit(kNop);
    CHECK(im.root() == 0, "adr x0: took 0x%llx as the root",
          static_cast<unsigned long long>(im.root()));
}

void testNoReference() {
    Image im;
    im.emit(adr(24, im.pc, kRoot));
    im.emit(kNop);
    searchBody(im, 24);
    CHECK(im.root() == 0, "no reference to the string: took 0x%llx",
          static_cast<unsigned long long>(im.root()));
}

}  // namespace

int main() {
    testAdrpAdd();
    testAdrNop();
    testMixed();
    testNegativeImmediate();
    testLocalAdrBetween();
    testOutOfRange();
    testX0IsNotTheRoot();
    testNoReference();
    if (g_failures != 0) {
        std::printf("%d check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
