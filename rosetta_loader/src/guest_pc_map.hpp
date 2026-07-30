#pragma once

#include <cstddef>
#include <cstdint>

// Recovery of a guest x86 program counter from a host ARM program counter in a
// process running under Rosetta 2.
//
// The Rosetta runtime (/usr/libexec/rosetta/runtime) keeps one
// CodeFragmentMetadata node per translated code fragment, intrusively linked
// into two binary search trees: one keyed on the fragment's ARM interval, one
// on its x86 interval.  The root of the ARM-keyed tree lives at a fixed offset
// in the runtime image.  Each translated node carries a DeltaCodedOffsetMap
// pairing offsets in the fragment's ARM output with offsets in the x86 input it
// was translated from, so an ARM pc inside a fragment resolves to a guest pc.
//
// Reads go through a caller-supplied callback, so the same code serves an
// in-process caller and one reading another task with mach_vm_read.  Nothing
// here writes to the target and nothing takes the runtime's tree lock, so a
// lookup can race a concurrent mutation of the tree.  Every step therefore
// validates what it reads and reports Status::Unavailable rather than trusting
// a partially observed structure; a sampler should drop such a sample.  An ARM
// pc belonging to no translated fragment is reported as Status::NotTranslated,
// which is also the test for "this thread is not currently executing translated
// guest code": outside translated code the guest register pinning does not
// hold either.

namespace guest_pc {

/// Reads `len` bytes at `addr` from the process being inspected into `dst`.
/// Must return false on any failure, including a short read.
using ReadFn = bool (*)(void* ctx, uint64_t addr, void* dst, size_t len);

struct Reader {
    ReadFn read = nullptr;
    void* ctx = nullptr;

    bool operator()(uint64_t addr, void* dst, size_t len) const {
        return read != nullptr && read(ctx, addr, dst, len);
    }
};

enum class Status : uint8_t {
    Resolved,       ///< the out parameter is valid
    NotTranslated,  ///< the ARM pc belongs to no translated fragment
    Unavailable,    ///< a read failed or a structure was inconsistent; drop the sample
};

/// Which of the three ways an ARM pc can fail to have a guest pc applied.  Only
/// meaningful alongside Status::NotTranslated, where it separates "the pc is in
/// Rosetta's own arm64 code" from "the pc is in translated output whose map does
/// not reach it", which a caller cannot tell apart from the status alone.
enum class NoGuestReason : uint8_t {
    Unknown = 0,
    NoFragment,           ///< in no fragment at all: not Rosetta's translated output
    RuntimeRoutines,      ///< a kind-0 fragment: the runtime's own code, wherever it is mapped
    BeforeFirstBoundary,  ///< inside a translated fragment, before its first mapped boundary
};

/// Offset of the ARM-keyed fragment tree root within the runtime image.
inline constexpr uint64_t kArmTreeRootOffset = 0x3ba08;

/// Field offsets within a CodeFragmentMetadata node.  The node is 0xA0 bytes
/// and everything a lookup needs lies in [kWindowBegin, kWindowEnd), which is
/// read in a single request per tree level.
namespace node {
inline constexpr uint64_t kArmLeft = 0x30;
inline constexpr uint64_t kArmRight = 0x38;
inline constexpr uint64_t kX86Begin = 0x48;
inline constexpr uint64_t kArmBegin = 0x58;
inline constexpr uint64_t kKind = 0x78;
inline constexpr uint64_t kMap = 0x80;
inline constexpr uint64_t kX86Size = 0x98;
inline constexpr uint64_t kArmSize = 0x9c;
inline constexpr uint64_t kWindowBegin = 0x30;
inline constexpr uint64_t kWindowEnd = 0xa0;
}  // namespace node

/// CodeFragmentMetadata::kind.  Values other than these two have not been
/// observed and are treated as unresolvable.
enum class FragmentKind : uint8_t {
    RuntimeRoutines = 0,  ///< a runtime code region: no x86 interval, no map
    Translated = 4,       ///< translated guest code, carries a DeltaCodedOffsetMap
};

/// The fields of one CodeFragmentMetadata node that a lookup needs.
struct Fragment {
    uint64_t node;       ///< address of the node itself
    uint64_t x86_begin;  ///< guest address the fragment was translated from
    uint64_t arm_begin;  ///< host address of the fragment's ARM code
    uint64_t map;        ///< DeltaCodedOffsetMap, or 0
    uint32_t x86_size;
    uint32_t arm_size;
    uint8_t kind;
};

/// Header of a DeltaCodedOffsetMap.  `rice_k` holds the per-field Golomb-Rice
/// parameters; the chunk array and the delta stream follow at the offsets
/// below, all relative to the header.
struct DeltaMapHeader {
    uint8_t rice_k[4];
    uint32_t field_04;
    uint32_t field_08;
    uint32_t field_0C;
    uint32_t total_size;  ///< header, chunks and stream together
    uint32_t chunk_count;
    uint32_t chunks_offset;
    uint32_t stream_offset;  ///< the stream runs to total_size
};

static_assert(sizeof(DeltaMapHeader) == 0x20, "DeltaMapHeader size mismatch");
static_assert(offsetof(DeltaMapHeader, total_size) == 0x10, "total_size offset mismatch");
static_assert(offsetof(DeltaMapHeader, chunk_count) == 0x14, "chunk_count offset mismatch");
static_assert(offsetof(DeltaMapHeader, chunks_offset) == 0x18, "chunks_offset offset mismatch");
static_assert(offsetof(DeltaMapHeader, stream_offset) == 0x1c, "stream_offset offset mismatch");

/// One entry of the chunk array: a restart point for the delta stream.  The
/// chunk gives the offsets to start from, the byte in the stream to start
/// reading at (always on a byte boundary), and how many steps it encodes.
struct DeltaMapChunk {
    uint32_t x86_offset;
    uint32_t arm_offset;
    uint32_t stream_byte;
    uint32_t insn_count;
};

static_assert(sizeof(DeltaMapChunk) == 16, "DeltaMapChunk size mismatch");

/// A boundary in a fragment's map: a point where the ARM output resynchronises
/// with an x86 instruction boundary.  Offsets are relative to the fragment's
/// arm_begin / x86_begin, and the ARM column advances in whole instructions.
struct MapPoint {
    uint32_t x86_offset;
    uint32_t arm_offset;
    uint32_t flags;
    bool exact;  ///< the queried ARM offset is this boundary, not inside its run
};

struct Resolution {
    uint64_t x86_pc;  ///< guest pc the ARM pc was translated from
    Fragment fragment;
    uint32_t arm_offset;  ///< queried ARM pc relative to fragment.arm_begin
    MapPoint point;
    /// Set on every Status::NotTranslated return, including the ones served from
    /// the cache.  Untouched otherwise.
    NoGuestReason reason = NoGuestReason::Unknown;
};

/// Walk the ARM-keyed tree for the fragment containing `armPc`.
Status lookupFragment(const Reader& reader, uint64_t runtimeBase, uint64_t armPc, Fragment& out);

/// Full chain: fragment lookup, then map decode.
Status resolve(const Reader& reader, uint64_t runtimeBase, uint64_t armPc, Resolution& out);

/// Remembers recent lookups so a repeat sample can skip the tree walk, which is
/// most of the cost: the walk spends one read per level, the map only three or
/// four.
///
/// There is no notification when the runtime frees a fragment, so entries are
/// not invalidated, they are VALIDATED on use: one read of the cached node must
/// still show the same interval, x86 base, map pointer and kind.  If any of that
/// moved, the entry is dropped and the lookup runs cold.  A cache keyed on the
/// ARM pc alone could not do this and would go quietly wrong, because a freed
/// fragment's ARM address is reused by a later translation.
///
/// Negative entries (an ARM pc in no translated fragment, the usual state of a
/// parked thread) cannot be validated that way, since a fragment may appear
/// there at any time.  They are re-checked periodically instead, so a stale
/// negative can only cost a missed sample, never a wrong guest pc.
///
/// Not thread safe: one cache per sampling thread.
class Cache {
public:
    /// Set associative: the set is chosen by the ARM page, the ways within it
    /// are scanned and matched on the fragment's ARM interval.  A fragment
    /// spanning several pages simply lands in several sets, which is harmless.
    ///
    /// Both dimensions were driven by measurement.  Fully associative with 32
    /// entries is enough for a microbenchmark but nowhere near a real game
    /// thread: its working set ran to thousands of fragments, misses grew
    /// without bound and the average lookup drifted from 5 us back up to 19 us
    /// as the cache fell behind.  A single way (direct mapped) thrashes the
    /// other way, since unrelated fragments share a page.
    static constexpr size_t kSets = 256;
    static constexpr size_t kWays = 8;
    static constexpr size_t kEntries = kSets * kWays;

    /// Re-walk a negative entry after this many hits.
    static constexpr uint32_t kNegativeRecheck = 64;

    void clear();

    struct Stats {
        uint64_t pc_hits = 0;        ///< same ARM pc as last time: one read
        uint64_t fragment_hits = 0;  ///< same fragment, map re-decoded
        uint64_t negative_hits = 0;
        uint64_t misses = 0;
        uint64_t stale = 0;  ///< validation failed, the fragment had moved
    };
    [[nodiscard]] const Stats& stats() const { return stats_; }

private:
    friend Status resolve(const Reader&, uint64_t, uint64_t, Resolution&, Cache&);

    struct Entry {
        Fragment fragment{};  ///< fragment.node == 0 marks a negative entry
        uint64_t arm_pc = 0;  ///< the exact pc this entry last answered
        uint64_t x86_pc = 0;  ///< and the answer, valid only for that pc
        MapPoint point{};
        uint64_t stamp = 0;    ///< for least-recently-used replacement
        uint32_t recheck = 0;  ///< negative entries only
        bool used = false;
    };

    /// The entry covering `armPc`, or the one in its set to replace.
    Entry& select(uint64_t armPc, bool& hit);

    Entry entries_[kEntries]{};
    Stats stats_;
    uint64_t clock_ = 0;
};

/// As `resolve`, but consulting and updating `cache`.  Returns exactly what the
/// uncached call would return.
Status resolve(const Reader& reader, uint64_t runtimeBase, uint64_t armPc, Resolution& out,
               Cache& cache);

/// Resolve an ARM offset against a complete in-memory copy of a map.  `mapSize`
/// must cover the whole map as declared by its header.  Fragments large enough
/// to carry a multi-megabyte map are resolved by `resolve` without ever holding
/// the whole thing; this entry point is for maps already in hand.
Status decodeMap(const uint8_t* map, size_t mapSize, uint32_t armOffset, MapPoint& out);

/// Decode every boundary encoded for chunk `chunkIndex`, including the chunk's
/// own starting pair, into `out`.  Returns the number of boundaries written, or
/// -1 if the map is malformed, the chunk's steps do not end where the next
/// chunk begins reading, or `outMax` is too small.
int decodeMapChunk(const uint8_t* map, size_t mapSize, uint32_t chunkIndex, MapPoint* out,
                   int outMax);

}  // namespace guest_pc
