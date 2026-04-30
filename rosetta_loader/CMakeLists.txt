add_executable(runtime_loader
    src/main.cpp
    src/macho_loader.cpp
    src/offset_finder.cpp
    src/sidecar.cpp
    src/stub_asm.cpp
)

target_link_libraries(runtime_loader PRIVATE rosetta_core)

add_custom_command(TARGET runtime_loader POST_BUILD
    COMMAND codesign -s - --entitlements
            "${CMAKE_CURRENT_SOURCE_DIR}/entitlements.plist"
            --force "$<TARGET_FILE:runtime_loader>"
    COMMENT "Signing runtime_loader with entitlements"
    VERBATIM
)
