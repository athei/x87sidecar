add_executable(aotinvoke
    src/main.cpp
    src/RosettaAotApi.cpp
)

target_link_libraries(aotinvoke PRIVATE rosetta_core)
