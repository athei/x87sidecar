#include "guest_pc_map.hpp"

#include <cstring>

namespace guest_pc {
namespace {

// The tree is unbalanced, so its depth is bounded only by the fragment count.
// The cap exists to stop a cycle introduced by a racing mutation, not to bound
// a legitimate walk.
constexpr int kMaxTreeDepth = 4096;

// A single fragment covering more than this is not plausible and indicates a
// torn read.
constexpr uint32_t kMaxFragmentSize = 64U << 20;

// A JIT fragment's map is a few hundred bytes, but an AOT module's runs to
// hundreds of kilobytes with hundreds of chunks, so the stream is read in
// windows rather than whole.
constexpr size_t kStreamWindow = 256;

// Guards the chunk array against a torn header.
constexpr uint32_t kMaxChunkCount = 1U << 20;

constexpr size_t kNodeWindow = node::kWindowEnd - node::kWindowBegin;

/// Least-significant-bit-first reader over the delta stream.  The stream is
/// either already in memory or pulled in windows through a Reader, so the same
/// decode serves both without holding a whole map.
class BitReader {
public:
    BitReader(const uint8_t* data, size_t size, size_t startByte)
        : mem_(data), size_(size), pos_(startByte) {}

    BitReader(const Reader& reader, uint64_t streamAddr, size_t size, size_t startByte)
        : size_(size), pos_(startByte), reader_(&reader), addr_(streamAddr) {}

    bool readBit(uint32_t& out) {
        uint8_t byte = 0;
        if (!byteAt(pos_, byte)) {
            return false;
        }
        out = (byte >> bit_) & 1U;
        if (++bit_ == 8) {
            bit_ = 0;
            pos_++;
        }
        return true;
    }

    /// Golomb-Rice: a unary prefix q (the count of 1 bits before the first 0),
    /// then k bits least significant first, giving (q << k) | r.  The prefix is
    /// long for large values (an x86 delta of 0x250 with k=2 spends 148 bits on
    /// it), so it is bounded only by the stream and by (q << k) overflowing.
    bool rice(unsigned k, uint32_t& out) {
        uint32_t q = 0;
        for (;;) {
            uint32_t bit = 0;
            if (!readBit(bit)) {
                return false;
            }
            if (bit == 0) {
                break;
            }
            if (++q > (0xFFFFFFFFU >> k)) {
                return false;
            }
        }
        uint32_t v = q << k;
        for (unsigned i = 0; i < k; i++) {
            uint32_t bit = 0;
            if (!readBit(bit)) {
                return false;
            }
            v |= bit << i;
        }
        out = v;
        return true;
    }

    bool raw32(uint32_t& out) {
        uint32_t v = 0;
        for (unsigned i = 0; i < 32; i++) {
            uint32_t bit = 0;
            if (!readBit(bit)) {
                return false;
            }
            v |= bit << i;
        }
        out = v;
        return true;
    }

    /// True when the decode ended where the next chunk starts reading, bar the
    /// padding: the stream is written in 32-bit words, so up to 31 zero bits
    /// can follow a chunk's last step.  Anything more means the step count and
    /// the stream disagree.
    [[nodiscard]] bool endsAt(size_t nextByte) {
        if (pos_ > nextByte || nextByte - pos_ > 4) {
            return false;
        }
        while (pos_ < nextByte) {
            uint32_t bit = 0;
            if (!readBit(bit)) {
                return false;
            }
            if (bit != 0) {
                return false;
            }
        }
        return true;
    }

private:
    /// Byte `idx` of the stream, pulling a window in when reading remotely.
    bool byteAt(size_t idx, uint8_t& out) {
        if (idx >= size_) {
            return false;
        }
        if (mem_ != nullptr) {
            out = mem_[idx];
            return true;
        }
        if (idx < winStart_ || idx >= winStart_ + winLen_) {
            const size_t want = size_ - idx < kStreamWindow ? size_ - idx : kStreamWindow;
            if (!(*reader_)(addr_ + idx, win_, want)) {
                return false;
            }
            winStart_ = idx;
            winLen_ = want;
        }
        out = win_[idx - winStart_];
        return true;
    }

    const uint8_t* mem_ = nullptr;
    size_t size_ = 0;
    size_t pos_ = 0;
    unsigned bit_ = 0;

    const Reader* reader_ = nullptr;
    uint64_t addr_ = 0;
    uint8_t win_[kStreamWindow] = {};
    size_t winStart_ = 0;
    size_t winLen_ = 0;
};

/// One advance along the delta stream.  The x86 column advances in bytes, the
/// ARM column in whole instructions.
bool step(BitReader& br, const uint8_t k[4], uint32_t& x86, uint32_t& arm, uint32_t& flags) {
    const unsigned k1 = k[1];
    const unsigned k2 = k[2];

    uint32_t d_arm = 0;
    uint32_t d_x86 = 0;
    uint32_t flag = 0;

    uint32_t a = 0;
    if (!br.rice(k2, a)) {
        return false;
    }
    if (a != 0) {
        d_arm = a;
        if (!br.rice(k1, d_x86)) {
            return false;
        }
    } else {
        uint32_t selector = 0;
        if (!br.rice(k2, selector)) {
            return false;
        }
        selector &= 0xff;
        if (selector == 0) {
            if (!br.raw32(d_arm) || !br.raw32(d_x86)) {
                return false;
            }
        } else {
            uint32_t lo = 0;
            uint32_t hi = 0;
            switch (selector) {
                case 1:
                    lo = 1;
                    break;
                case 2: {
                    lo = 1;
                    uint32_t extra = 0;
                    if (!br.rice(k2, extra)) {
                        return false;
                    }
                    hi = (extra << 8) & 0xFF00U;
                    break;
                }
                case 3:
                    lo = 2;
                    break;
                case 4:
                    lo = 3;
                    break;
                default:
                    break;
            }
            uint32_t a2 = 0;
            if (!br.rice(k2, a2)) {
                return false;
            }
            if (a2 != 0) {
                d_arm = a2;
                if (!br.rice(k1, d_x86)) {
                    return false;
                }
            } else if (!br.raw32(d_arm) || !br.raw32(d_x86)) {
                return false;
            }
            flag = hi | lo;
        }
    }

    if (d_arm == 0 || d_arm > kMaxFragmentSize / 4) {
        return false;  // the ARM column is strictly increasing
    }
    x86 += d_x86;
    arm += d_arm * 4;
    flags = flag;
    return true;
}

struct MapView {
    DeltaMapHeader header;
    const DeltaMapChunk* chunks;
    const uint8_t* stream;
    size_t stream_size;
};

/// Structural validation of a map header, independent of where it is stored.
bool checkHeader(const DeltaMapHeader& h) {
    // Only the two-parameter encoding has been observed.  The fourth descriptor
    // byte selects a variant this decoder does not implement, so refuse a map
    // that sets it rather than decode it wrongly.  field_04 is read only when
    // that byte is set, so its value is ignored here; single-instruction
    // fragments do set it.
    if (h.rice_k[3] != 0 || h.field_08 != 0 || h.field_0C != 0) {
        return false;
    }
    if (h.rice_k[1] > 31 || h.rice_k[2] > 31) {
        return false;
    }
    if (h.total_size < sizeof(h) || h.chunk_count == 0 || h.chunk_count > kMaxChunkCount) {
        return false;
    }
    if (h.chunks_offset < sizeof(h) || h.stream_offset > h.total_size ||
        h.stream_offset < h.chunks_offset) {
        return false;
    }
    const uint64_t chunksEnd = static_cast<uint64_t>(h.chunks_offset) +
                               (static_cast<uint64_t>(h.chunk_count) * sizeof(DeltaMapChunk));
    return chunksEnd <= h.stream_offset;
}

bool parseMap(const uint8_t* map, size_t mapSize, MapView& out) {
    if (map == nullptr || mapSize < sizeof(DeltaMapHeader)) {
        return false;
    }
    DeltaMapHeader h{};
    memcpy(&h, map, sizeof(h));
    if (!checkHeader(h) || h.total_size > mapSize) {
        return false;
    }
    out.header = h;
    out.chunks = reinterpret_cast<const DeltaMapChunk*>(map + h.chunks_offset);
    out.stream = map + h.stream_offset;
    out.stream_size = h.total_size - h.stream_offset;
    return true;
}

/// Index of the last chunk whose ARM offset is at or before `armOffset`.
/// `fetch` reads chunk `i`; it fails only when the underlying read fails.
template <typename Fetch>
bool findChunk(uint32_t chunkCount, uint32_t armOffset, Fetch fetch, uint32_t& out) {
    uint32_t lo = 0;
    uint32_t hi = chunkCount;
    while (hi - lo > 1) {
        const uint32_t mid = lo + ((hi - lo) / 2);
        DeltaMapChunk c{};
        if (!fetch(mid, c)) {
            return false;
        }
        if (c.arm_offset <= armOffset) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    out = lo;
    return true;
}

/// Step the stream from a chunk's starting pair up to `armOffset`, keeping the
/// last boundary at or before it.  Stops early once the ARM column passes the
/// target; an ARM offset past the chunk's last step yields that last boundary.
Status walkToTarget(BitReader& br, const uint8_t k[4], const DeltaMapChunk& chunk,
                    uint32_t armOffset, MapPoint& out) {
    uint32_t x86 = chunk.x86_offset;
    uint32_t arm = chunk.arm_offset;
    if (armOffset < arm) {
        return Status::NotTranslated;  // before the first mapped boundary
    }
    MapPoint best{x86, arm, 0, arm == armOffset};

    for (uint32_t i = 0; i < chunk.insn_count; i++) {
        const uint32_t prevArm = arm;
        uint32_t flags = 0;
        if (!step(br, k, x86, arm, flags)) {
            return Status::Unavailable;
        }
        // Only the ARM column is ordered.  The x86 column follows the guest's
        // control flow, so a delta can be negative (encoded as a raw 32-bit
        // two's complement value) and the running offset wraps deliberately.
        if (arm <= prevArm) {
            return Status::Unavailable;
        }
        if (arm > armOffset) {
            break;
        }
        best = MapPoint{x86, arm, flags, arm == armOffset};
    }

    out = best;
    return Status::Resolved;
}

/// Fill a Fragment from a node window read at node::kWindowBegin.
void parseNode(const uint8_t* win, uint64_t nodeAddr, Fragment& out) {
    const auto field = [win](uint64_t off) { return win + (off - node::kWindowBegin); };
    out.node = nodeAddr;
    memcpy(&out.arm_begin, field(node::kArmBegin), sizeof(out.arm_begin));
    memcpy(&out.arm_size, field(node::kArmSize), sizeof(out.arm_size));
    memcpy(&out.x86_begin, field(node::kX86Begin), sizeof(out.x86_begin));
    memcpy(&out.map, field(node::kMap), sizeof(out.map));
    memcpy(&out.x86_size, field(node::kX86Size), sizeof(out.x86_size));
    out.kind = *field(node::kKind);
}

bool contains(const Fragment& frag, uint64_t armPc) {
    return frag.node != 0 && armPc >= frag.arm_begin && armPc < frag.arm_begin + frag.arm_size;
}

/// One read that proves a cached fragment is still the fragment it was.  A
/// fragment that was freed and replaced fails on the interval, the x86 base or
/// the map pointer.
bool nodeUnchanged(const Reader& reader, const Fragment& cached) {
    uint8_t win[kNodeWindow];
    if (!reader(cached.node + node::kWindowBegin, win, sizeof(win))) {
        return false;
    }
    Fragment now{};
    parseNode(win, cached.node, now);
    return now.arm_begin == cached.arm_begin && now.arm_size == cached.arm_size &&
           now.x86_begin == cached.x86_begin && now.x86_size == cached.x86_size &&
           now.map == cached.map && now.kind == cached.kind;
}

/// Decode `armPc` against a fragment already known to be current.
Status resolveInFragment(const Reader& reader, const Fragment& frag, uint64_t armPc,
                         Resolution& out) {
    if (frag.kind != static_cast<uint8_t>(FragmentKind::Translated)) {
        out.reason = NoGuestReason::RuntimeRoutines;
        return Status::NotTranslated;
    }
    if (frag.map == 0 || frag.x86_size == 0) {
        return Status::Unavailable;
    }

    DeltaMapHeader header{};
    if (!reader(frag.map, &header, sizeof(header)) || !checkHeader(header)) {
        return Status::Unavailable;
    }

    // An AOT module's map is hundreds of kilobytes with hundreds of chunks, so
    // only the chunks the binary search touches and the stream window the
    // decode reaches are ever read.
    const uint32_t armOffset = static_cast<uint32_t>(armPc - frag.arm_begin);
    const uint64_t chunksAddr = frag.map + header.chunks_offset;
    uint32_t index = 0;
    if (!findChunk(
            header.chunk_count, armOffset,
            [&reader, chunksAddr](uint32_t i, DeltaMapChunk& c) {
                return reader(chunksAddr + (static_cast<uint64_t>(i) * sizeof(DeltaMapChunk)), &c,
                              sizeof(c));
            },
            index)) {
        return Status::Unavailable;
    }

    DeltaMapChunk chunk{};
    if (!reader(chunksAddr + (static_cast<uint64_t>(index) * sizeof(DeltaMapChunk)), &chunk,
                sizeof(chunk))) {
        return Status::Unavailable;
    }
    const size_t streamSize = header.total_size - header.stream_offset;
    if (chunk.stream_byte > streamSize) {
        return Status::Unavailable;
    }

    MapPoint point{};
    BitReader br(reader, frag.map + header.stream_offset, streamSize, chunk.stream_byte);
    const Status decoded = walkToTarget(br, header.rice_k, chunk, armOffset, point);
    if (decoded != Status::Resolved) {
        if (decoded == Status::NotTranslated) {
            out.reason = NoGuestReason::BeforeFirstBoundary;
        }
        return decoded;
    }
    // The map's last boundary sits at the fragment's x86 end: an ARM pc there
    // has retired every guest instruction in the fragment, so the guest pc is
    // the first byte past it.  Anything beyond that is a torn read.
    if (point.x86_offset > frag.x86_size) {
        return Status::Unavailable;
    }

    out.fragment = frag;
    out.arm_offset = armOffset;
    out.point = point;
    out.x86_pc = frag.x86_begin + point.x86_offset;
    return Status::Resolved;
}

}  // namespace

Status lookupFragment(const Reader& reader, uint64_t runtimeBase, uint64_t armPc, Fragment& out) {
    uint64_t nodeAddr = 0;
    if (!reader(runtimeBase + kArmTreeRootOffset, &nodeAddr, sizeof(nodeAddr))) {
        return Status::Unavailable;
    }

    for (int depth = 0; depth < kMaxTreeDepth; depth++) {
        if (nodeAddr == 0) {
            return Status::NotTranslated;
        }
        if (nodeAddr < 0x1000 || (nodeAddr & 7) != 0) {
            return Status::Unavailable;
        }

        uint8_t win[kNodeWindow];
        if (!reader(nodeAddr + node::kWindowBegin, win, sizeof(win))) {
            return Status::Unavailable;
        }
        const auto field = [&win](uint64_t off) { return win + (off - node::kWindowBegin); };

        Fragment frag{};
        parseNode(win, nodeAddr, frag);
        if (frag.arm_size == 0 || frag.arm_size > kMaxFragmentSize) {
            return Status::Unavailable;
        }

        if (armPc < frag.arm_begin) {
            memcpy(&nodeAddr, field(node::kArmLeft), sizeof(nodeAddr));
            continue;
        }
        if (armPc >= frag.arm_begin + frag.arm_size) {
            memcpy(&nodeAddr, field(node::kArmRight), sizeof(nodeAddr));
            continue;
        }

        out = frag;
        return Status::Resolved;
    }
    return Status::Unavailable;
}

Status decodeMap(const uint8_t* map, size_t mapSize, uint32_t armOffset, MapPoint& out) {
    MapView v{};
    if (!parseMap(map, mapSize, v)) {
        return Status::Unavailable;
    }

    uint32_t index = 0;
    findChunk(
        v.header.chunk_count, armOffset,
        [&v](uint32_t i, DeltaMapChunk& c) {
            c = v.chunks[i];
            return true;
        },
        index);

    const DeltaMapChunk& chunk = v.chunks[index];
    if (chunk.stream_byte > v.stream_size) {
        return Status::Unavailable;
    }
    BitReader br(v.stream, v.stream_size, chunk.stream_byte);
    return walkToTarget(br, v.header.rice_k, chunk, armOffset, out);
}

int decodeMapChunk(const uint8_t* map, size_t mapSize, uint32_t chunkIndex, MapPoint* out,
                   int outMax) {
    MapView v{};
    if (!parseMap(map, mapSize, v) || chunkIndex >= v.header.chunk_count || out == nullptr) {
        return -1;
    }

    const DeltaMapChunk& chunk = v.chunks[chunkIndex];
    if (static_cast<uint64_t>(chunk.insn_count) + 1 > static_cast<uint64_t>(outMax)) {
        return -1;
    }
    if (chunk.stream_byte > v.stream_size) {
        return -1;
    }
    // A chunk's steps run up to where the next chunk starts reading, or to the
    // end of the stream for the last one.
    const size_t nextByte = chunkIndex + 1 < v.header.chunk_count
                                ? v.chunks[chunkIndex + 1].stream_byte
                                : v.stream_size;

    uint32_t x86 = chunk.x86_offset;
    uint32_t arm = chunk.arm_offset;
    int count = 0;
    out[count++] = MapPoint{x86, arm, 0, true};

    BitReader br(v.stream, v.stream_size, chunk.stream_byte);
    for (uint32_t i = 0; i < chunk.insn_count; i++) {
        const uint32_t prevArm = arm;
        uint32_t flags = 0;
        if (!step(br, v.header.rice_k, x86, arm, flags)) {
            return -1;
        }
        if (arm <= prevArm) {
            return -1;
        }
        out[count++] = MapPoint{x86, arm, flags, true};
    }
    if (!br.endsAt(nextByte)) {
        return -1;
    }
    return count;
}

Status resolve(const Reader& reader, uint64_t runtimeBase, uint64_t armPc, Resolution& out) {
    Fragment frag{};
    const Status found = lookupFragment(reader, runtimeBase, armPc, frag);
    if (found != Status::Resolved) {
        if (found == Status::NotTranslated) {
            out.reason = NoGuestReason::NoFragment;
        }
        return found;
    }
    return resolveInFragment(reader, frag, armPc, out);
}

void Cache::clear() {
    for (auto& entry : entries_) {
        entry = Entry{};
    }
    clock_ = 0;
}

Cache::Entry& Cache::select(uint64_t armPc, bool& hit) {
    clock_++;
    Entry* const set = &entries_[((armPc >> 12) % kSets) * kWays];
    Entry* victim = set;
    for (Entry* it = set; it != set + kWays; ++it) {
        Entry& entry = *it;
        if (entry.used) {
            const bool match =
                entry.fragment.node != 0 ? contains(entry.fragment, armPc) : entry.arm_pc == armPc;
            if (match) {
                entry.stamp = clock_;
                hit = true;
                return entry;
            }
        }
        if (!entry.used || entry.stamp < victim->stamp) {
            victim = &entry;
        }
    }
    hit = false;
    victim->stamp = clock_;
    return *victim;
}

Status resolve(const Reader& reader, uint64_t runtimeBase, uint64_t armPc, Resolution& out,
               Cache& cache) {
    bool hit = false;
    Cache::Entry& entry = cache.select(armPc, hit);

    if (hit && entry.fragment.node != 0) {
        if (nodeUnchanged(reader, entry.fragment)) {
            // A runtime region carries no guest code; that answer is as cheap
            // to serve from cache as a translated one, and parked threads sit
            // in such regions almost all the time.
            if (entry.fragment.kind != static_cast<uint8_t>(FragmentKind::Translated)) {
                cache.stats_.negative_hits++;
                out.reason = NoGuestReason::RuntimeRoutines;
                return Status::NotTranslated;
            }
            if (entry.arm_pc == armPc) {
                // Same pc in the same fragment: the decode cannot have changed.
                cache.stats_.pc_hits++;
                out.fragment = entry.fragment;
                out.arm_offset = static_cast<uint32_t>(armPc - entry.fragment.arm_begin);
                out.point = entry.point;
                out.x86_pc = entry.x86_pc;
                return Status::Resolved;
            }
            const Status status = resolveInFragment(reader, entry.fragment, armPc, out);
            if (status != Status::Unavailable) {
                cache.stats_.fragment_hits++;
                entry.arm_pc = armPc;
                entry.x86_pc = out.x86_pc;
                entry.point = out.point;
                return status;
            }
            // Fall through to a cold lookup rather than trust a partial answer.
        } else {
            cache.stats_.stale++;
            entry = Cache::Entry{};
        }
    } else if (hit && entry.recheck < Cache::kNegativeRecheck) {
        // An ARM pc in no fragment at all cannot be validated, since one may
        // appear there at any time, so re-walk periodically.  A stale negative
        // costs a dropped sample, never a wrong answer.
        entry.recheck++;
        cache.stats_.negative_hits++;
        out.reason = NoGuestReason::NoFragment;
        return Status::NotTranslated;
    }

    cache.stats_.misses++;
    Fragment frag{};
    const Status found = lookupFragment(reader, runtimeBase, armPc, frag);
    if (found == Status::NotTranslated) {
        const uint64_t stamp = entry.stamp;
        entry = Cache::Entry{};
        entry.used = true;
        entry.arm_pc = armPc;
        entry.stamp = stamp;
        out.reason = NoGuestReason::NoFragment;
        return found;
    }
    if (found != Status::Resolved) {
        return found;
    }

    const Status status = resolveInFragment(reader, frag, armPc, out);
    if (status != Status::Unavailable) {
        const uint64_t stamp = entry.stamp;
        entry = Cache::Entry{};
        entry.used = true;
        entry.fragment = frag;
        entry.arm_pc = armPc;
        entry.x86_pc = status == Status::Resolved ? out.x86_pc : 0;
        entry.point = out.point;
        entry.stamp = stamp;
    }
    return status;
}

}  // namespace guest_pc
