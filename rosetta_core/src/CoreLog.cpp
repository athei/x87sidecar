#include "rosetta_core/CoreLog.h"

#include <cstdio>


// Default sink: write to stdout. Override in each binary's entry point.
static void default_log(const char* msg) {
    printf("%s\n", msg);
}

core_log_fn_t g_core_log_fn = default_log;
