add_executable(profile_analyze
    profile_analyze.cpp
)

target_link_libraries(profile_analyze PRIVATE rosetta_core)
