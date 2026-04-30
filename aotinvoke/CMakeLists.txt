add_executable(aotinvoke
    src/main.cpp
    src/RosettaAotApi.cpp
)

target_include_directories(aotinvoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(aotinvoke PRIVATE rosetta_core)
