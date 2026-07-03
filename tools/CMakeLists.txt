add_executable(profile_analyze
    profile_analyze.cpp
)

target_link_libraries(profile_analyze PRIVATE rosetta_core)

add_executable(rollback_diff
    rollback_diff.cpp
)

target_link_libraries(rollback_diff PRIVATE rosetta_core)

add_executable(ir_pressure_replay
    ir_pressure_replay.cpp
)
target_link_libraries(ir_pressure_replay PRIVATE rosetta_core)
