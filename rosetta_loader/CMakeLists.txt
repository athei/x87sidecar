add_executable(x87sidecar
    src/main.cpp
    src/macho_loader.cpp
    src/offset_finder.cpp
    src/sidecar.cpp
    src/stub_asm.cpp
)

target_link_libraries(x87sidecar PRIVATE rosetta_core)

add_custom_command(TARGET x87sidecar POST_BUILD
    COMMAND codesign -s - --entitlements
            "${CMAKE_CURRENT_SOURCE_DIR}/entitlements.plist"
            --force "$<TARGET_FILE:x87sidecar>"
    COMMENT "Signing x87sidecar with entitlements"
    VERBATIM
)
