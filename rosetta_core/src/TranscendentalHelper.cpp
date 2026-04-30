#include "rosetta_core/TranscendentalHelper.h"

namespace rosetta_core {

namespace {
uint64_t g_helper_addr = 0;
}

void set_transcendental_helper_addr(uint64_t addr) {
    g_helper_addr = addr;
}

uint64_t get_transcendental_helper_addr() {
    return g_helper_addr;
}

}  // namespace rosetta_core
