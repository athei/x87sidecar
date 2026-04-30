#include "rosetta_core/TranscendentalHelper.h"

namespace rosetta_core {

namespace {
uint64_t g_helper_addr = 0;
uint64_t g_constants_addr = 0;
}

void set_transcendental_helper_addr(uint64_t addr) {
    g_helper_addr = addr;
}

uint64_t get_transcendental_helper_addr() {
    return g_helper_addr;
}

void set_transcendental_constants_addr(uint64_t addr) {
    g_constants_addr = addr;
}

uint64_t get_transcendental_constants_addr() {
    return g_constants_addr;
}

}  // namespace rosetta_core
