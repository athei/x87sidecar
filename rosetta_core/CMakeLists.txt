set(ROSETTA_CORE_SOURCES
    src/CoreConfig.cpp
    src/CoreLog.cpp
    src/CustomTranslationHook.cpp
    src/hook.cpp
    src/AssemblerHelpers.cpp
    src/AssemblerBuffer.cpp
    src/Opcode.cpp
    src/TranslatorHelpers.cpp
    src/Translator.cpp
    src/TranslatorX87.cpp
    src/TranslatorX87Fusion.cpp
    src/TranslatorX87Helpers.cpp
    src/X87Cache.cpp
    src/X87IRBuild.cpp
    src/X87IROptimize.cpp
    src/X87IRLower.cpp
)

# Single canonical build — the dylib variant (`rosetta_core_runtime` with
# ROSETTA_RUNTIME=1, RuntimeLibC shims, hidden visibility, nostdlib) was
# dropped along with libRuntimeRosettax87 when the loader switched to an
# inline IPC stub installed into stock libRosettaRuntime's __TEXT.
add_library(rosetta_core STATIC ${ROSETTA_CORE_SOURCES})
target_include_directories(rosetta_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(rosetta_core PUBLIC cxx_std_23)
target_compile_options(rosetta_core PRIVATE -g -O2)
target_link_libraries(rosetta_core PUBLIC rosetta_config)
set_property(TARGET rosetta_core PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
