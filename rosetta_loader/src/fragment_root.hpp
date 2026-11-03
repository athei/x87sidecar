#pragma once

#include <cstddef>
#include <cstdint>

// The root of the runtime's ARM-keyed code-fragment tree, which the sampler
// walks, as an offset in /usr/libexec/rosetta/runtime. Split from OffsetFinder
// so the search can be fed synthetic bytes in tests.
//
// The lookup that walks the tree panics with "cannot locate code fragment for
// arm address" when a pc is in no fragment; `panicString` is that string's
// offset. The root is the global the lookup materialises last before the
// string, in a register other than x0. Both addresses come either from
// `adrp xN; add xN, xN, #lo` or, where the linker relaxed that pair, from
// `adr xN; nop`. The root is a global in __DATA or __bss, so it lies past
// __text, within 4 MB of its end. Walking back from the string, an adr aimed
// elsewhere is passed over, and the first adrp/add pair ends the search.
//
// `image` holds the whole file and `[textBegin, textEnd)` is __text, as file
// offsets, which equal the image's vm offsets. Returns 0 when not found.
auto findArmTreeRoot(const std::uint8_t* image, std::size_t size, std::uint64_t textBegin,
                     std::uint64_t textEnd, std::uint64_t panicString) -> std::uint64_t;
