add_library(rosetta_core STATIC
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
    src/X87IRLower.cpp
    src/X87IROptimize.cpp
)

target_include_directories(rosetta_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(rosetta_core PUBLIC rosetta_config)
