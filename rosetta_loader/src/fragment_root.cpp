#include "fragment_root.hpp"

#include <algorithm>
#include <cstring>
#include <vector>

namespace {

auto insnAt(const std::uint8_t* image, std::uint64_t off) -> std::uint32_t {
    std::uint32_t v = 0;
    std::memcpy(&v, image + off, sizeof(v));
    return v;
}

// The 21-bit signed immediate of adr and adrp: immhi in bits 23-5, immlo in
// bits 30-29.
auto pcRelImm(std::uint32_t insn) -> std::int64_t {
    const std::uint32_t immlo = (insn >> 29) & 0x3;
    const std::uint32_t immhi = (insn >> 5) & 0x7FFFF;
    std::int64_t imm = (static_cast<std::int64_t>(immhi) << 2) | immlo;
    if (imm & (1LL << 20)) {
        imm -= 1LL << 21;
    }
    return imm;
}

// adr: op (bit 31) 0, bits 28-24 10000.
auto isAdr(std::uint32_t insn) -> bool {
    return (insn & 0x9F000000) == 0x10000000;
}
// adrp: op 1, bits 28-24 10000.
auto isAdrp(std::uint32_t insn) -> bool {
    return (insn & 0x9F000000) == 0x90000000;
}
// add xd, xn, #imm12 (no shift)
auto isAddImm(std::uint32_t insn) -> bool {
    return (insn & 0xFFC00000) == 0x91000000;
}

auto adrTarget(std::uint32_t insn, std::uint64_t off) -> std::uint64_t {
    return off + static_cast<std::uint64_t>(pcRelImm(insn));
}
auto adrpTarget(std::uint32_t insn, std::uint64_t off) -> std::uint64_t {
    return (off & ~0xFFFULL) + static_cast<std::uint64_t>(pcRelImm(insn) * 4096);
}

// Offsets of the instructions that finish materialising `target`: the add of
// an adrp/add pair, or an adr.
auto refsTo(const std::uint8_t* image, std::uint64_t textBegin, std::uint64_t textEnd,
            std::uint64_t target) -> std::vector<std::uint64_t> {
    std::vector<std::uint64_t> refs;
    for (std::uint64_t off = textBegin; off + 4 <= textEnd; off += 4) {
        const std::uint32_t insn = insnAt(image, off);
        if (isAdr(insn)) {
            if (adrTarget(insn, off) == target) {
                refs.push_back(off);
            }
            continue;
        }
        if (!isAddImm(insn)) {
            continue;
        }
        const std::uint32_t rn = (insn >> 5) & 31;
        const std::uint64_t lo12 = (insn >> 10) & 0xFFF;
        // The adrp is normally the previous instruction; allow a few in between.
        for (std::uint64_t back = 4; back <= 32 && off >= textBegin + back; back += 4) {
            const std::uint32_t adrp = insnAt(image, off - back);
            if (isAdrp(adrp) && (adrp & 31) == rn) {
                if (adrpTarget(adrp, off - back) + lo12 == target) {
                    refs.push_back(off);
                }
                break;
            }
        }
    }
    return refs;
}

}  // namespace

auto findArmTreeRoot(const std::uint8_t* image, std::size_t size, std::uint64_t textBegin,
                     std::uint64_t textEnd, std::uint64_t panicString) -> std::uint64_t {
    textEnd = std::min<std::uint64_t>(textEnd, size);
    if (image == nullptr || textBegin >= textEnd) {
        return 0;
    }
    const auto isGlobal = [textEnd](std::uint64_t target) {
        return target >= textEnd && target < textEnd + 0x400000;
    };
    for (const std::uint64_t ref : refsTo(image, textBegin, textEnd, panicString)) {
        // Walk back from the string, skipping x0, which carries the string
        // itself. An `adr xN` whose target is a global past __text is the
        // root; one aimed anywhere else, such as a jump table in __text, is
        // passed over. The first `adrp xN; add xN, xN, #lo` pair ends the
        // walk: the root if its target is a global, else nothing.
        for (std::uint64_t back = 4; back <= 64 * 4 && ref >= textBegin + back; back += 4) {
            const std::uint64_t off = ref - back;
            const std::uint32_t insn = insnAt(image, off);
            if (isAdr(insn)) {
                if ((insn & 31) != 0 && isGlobal(adrTarget(insn, off))) {
                    return adrTarget(insn, off);
                }
                continue;
            }
            if (!isAddImm(insn) || (insn & 31) != ((insn >> 5) & 31) || (insn & 31) == 0 ||
                off < textBegin + 4) {
                continue;
            }
            const std::uint32_t adrp = insnAt(image, off - 4);
            if (!isAdrp(adrp) || (adrp & 31) != (insn & 31)) {
                continue;
            }
            const std::uint64_t target = adrpTarget(adrp, off - 4) + ((insn >> 10) & 0xFFF);
            if (isGlobal(target)) {
                return target;
            }
            break;
        }
    }
    return 0;
}
