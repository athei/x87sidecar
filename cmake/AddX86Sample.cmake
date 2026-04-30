# add_x86_sample(<name>)
#
# Compile <name>.c in the current source dir as an x86_64 binary that
# exercises the x87 FPU under Rosetta 2 / our JIT. The host build is arm64
# so this is a cross-compile; we go through /usr/bin/clang directly because
# Apple's clang is the only one that reliably handles -arch x86_64 here, and
# we don't want any of the host project's compile flags applied to it.
#
# Output directory: ${X86_SAMPLE_OUTPUT_DIR} if set in the calling scope,
# otherwise ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}.
function(add_x86_sample name)
    set(out_dir "${X86_SAMPLE_OUTPUT_DIR}")
    if(NOT out_dir)
        set(out_dir "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
    endif()
    set(src "${CMAKE_CURRENT_SOURCE_DIR}/${name}.c")
    set(out "${out_dir}/${name}")
    add_custom_command(
        OUTPUT  "${out}"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${out_dir}"
        COMMAND /usr/bin/clang -arch x86_64 -O2 -o "${out}" "${src}"
        DEPENDS "${src}"
        COMMENT "Building x86_64 ${name}"
        VERBATIM
    )
    add_custom_target(${name}_x86_64 ALL DEPENDS "${out}")
endfunction()
