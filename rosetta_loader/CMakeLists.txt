add_executable(rosettax87
    src/main.cpp
    src/macho_loader.cpp
    src/offset_finder.cpp
    src/sidecar.cpp
    src/stub_asm.cpp
)

target_link_libraries(rosettax87 PRIVATE rosetta_core)

add_custom_command(TARGET rosettax87 POST_BUILD
    COMMAND codesign -s - --entitlements
            "${CMAKE_CURRENT_SOURCE_DIR}/entitlements.plist"
            --force "$<TARGET_FILE:rosettax87>"
    COMMENT "Signing rosettax87 with entitlements"
    VERBATIM
)
