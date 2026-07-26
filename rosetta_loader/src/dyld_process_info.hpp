#ifndef X87SIDECAR_DYLD_PROCESS_INFO_HPP
#define X87SIDECAR_DYLD_PROCESS_INFO_HPP

#include <mach/mach.h>
#include <uuid/uuid.h>

#include <cstdint>

// dyld's cross-task image list. Private libSystem SPI: it is how the loader
// finds the Rosetta runtime among the tracee's images, and how the sampler
// names the Mach-O half of the guest's module map. There is no public
// equivalent that works on another task, and dlopen'ing dyld's own list would
// describe the wrong process.
//
// Reading it does not stop the target: the caller passes a task port and dyld
// walks the target's image list out of its memory.
using DyldProcessInfo = struct dyld_process_info_base*;

extern "C" DyldProcessInfo _dyld_process_info_create(task_t task, uint64_t timestamp,
                                                     kern_return_t* kernelError);
extern "C" void _dyld_process_info_for_each_image(DyldProcessInfo info,
                                                  void (^callback)(uint64_t machHeaderAddress,
                                                                   const uuid_t uuid,
                                                                   const char* path));
extern "C" void _dyld_process_info_release(DyldProcessInfo info);

#endif  // X87SIDECAR_DYLD_PROCESS_INFO_HPP
