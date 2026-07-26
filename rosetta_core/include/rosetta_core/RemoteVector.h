#pragma once

#include <cstdint>

/// A `std::vector` living in the tracee.
///
/// Stock stores several of its per-translation tables this way: three raw
/// pointers, `{begin, end, capacity_end}`, with the element count derived as
/// `(end - begin) / sizeof(T)`.
///
/// The pointers are tracee-side VAs. The sidecar runs out of process, so they
/// are only ever followed with `mach_vm_read`, never dereferenced directly —
/// hence plain `uint64_t` rather than `T*`. The element type is named at each
/// use site.
struct RemoteVector {
    uint64_t begin;
    uint64_t end;
    uint64_t cap;

    /// Byte span of the live elements. Empty vectors have `begin == end == 0`.
    uint64_t size_bytes() const { return end - begin; }

    uint64_t count(uint64_t elem_size) const {
        return elem_size == 0 ? 0 : size_bytes() / elem_size;
    }
};

static_assert(sizeof(RemoteVector) == 0x18, "RemoteVector size mismatch");
