# Generate the MIG mach_exc exception server (message ids 2405-2407,
# MACH_EXCEPTION_CODES) so the loader can receive ptrace debug events on a Mach
# exception port instead of via the deprecated PT_ATTACH/waitpid signal path.
# See src/mach_exception.cpp.
execute_process(
    COMMAND xcrun --sdk macosx --show-sdk-path
    OUTPUT_VARIABLE MACOS_SDK_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(MACH_EXC_SERVER "${CMAKE_CURRENT_BINARY_DIR}/mach_excServer.c")
set(MACH_EXC_SHEADER "${CMAKE_CURRENT_BINARY_DIR}/mach_excServer.h")

add_custom_command(
    OUTPUT ${MACH_EXC_SERVER} ${MACH_EXC_SHEADER}
    COMMAND xcrun mig
            -arch arm64
            -isysroot "${MACOS_SDK_PATH}"
            -server ${MACH_EXC_SERVER}
            -sheader ${MACH_EXC_SHEADER}
            -user /dev/null
            -header /dev/null
            "${MACOS_SDK_PATH}/usr/include/mach/mach_exc.defs"
    DEPENDS "${MACOS_SDK_PATH}/usr/include/mach/mach_exc.defs"
    COMMENT "Generating MIG mach_exc exception server"
    VERBATIM
)

add_executable(x87sidecar
    src/main.cpp
    src/macho_loader.cpp
    src/mach_exception.cpp
    src/offset_finder.cpp
    src/sidecar.cpp
    src/stub_asm.cpp
    ${MACH_EXC_SERVER}
)

# mach_exception.cpp includes the generated mach_excServer.h from the build dir.
target_include_directories(x87sidecar PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(x87sidecar PRIVATE rosetta_core)

add_custom_command(TARGET x87sidecar POST_BUILD
    COMMAND codesign -s - --entitlements
            "${CMAKE_CURRENT_SOURCE_DIR}/entitlements.plist"
            --force "$<TARGET_FILE:x87sidecar>"
    COMMENT "Signing x87sidecar with entitlements"
    VERBATIM
)
