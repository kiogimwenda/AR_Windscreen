# FlatBuffers schema generation — see docs/BUILD_GUIDE.md Part 3.5.
#
# ---------------------------------------------------------------------------------------------
# What this does, if FlatBuffers is unfamiliar:
#
# A .fbs file declares message types. `flatc --cpp` compiles each one into a C++ header. That
# header contains two representations of every table:
#
#   ActuationRequest   — a read-only VIEW over a serialized byte buffer. Reading a field is a
#                        pointer offset, with no parsing or copying. This is what you would
#                        send over a socket or write to a file for later replay.
#   ActuationRequestT  — (only with --gen-object-api) a plain, mutable C++ struct with ordinary
#                        members, std::vector and std::string. Pack() and UnPack() convert
#                        between the two.
#
# Host subsystems pass the T structs across the Part 5.3 ring buffers. Serialising and parsing a
# buffer for every hop between two threads of the same process would buy nothing. The byte
# representation stays available for the day these messages need recording or replaying
# (Part 13.2's replay tooling), with no second definition of any type.
#
# add_custom_command declares HOW to produce each header. CMake reruns flatc only when that .fbs
# changes. The generated headers go into the build tree, never the source tree: Part 3.5 forbids
# committing them, and a stale committed copy silently overriding a newer schema is exactly the
# bug that rule exists to prevent.
# ---------------------------------------------------------------------------------------------
#
# ar_add_schema_library(<name> <schema_dir>)
#   Creates INTERFACE library <name>. Linking it gives a target the generated headers, reachable as
#   "ar_drive_assist/common/schemas/<stem>_generated.h", and guarantees flatc runs first.

function(ar_add_schema_library name schema_dir)
    find_program(FLATC_EXECUTABLE flatc REQUIRED)

    # flatc and the FlatBuffers runtime headers must be the SAME version. Every generated header
    # static_asserts this, but the failure it produces ("Non-compatible flatbuffers version
    # included") names neither version nor either file. On this machine it is a live risk: OSRM
    # installs its own FlatBuffers 24.x headers into /usr/local/include, which the compiler
    # searches before Debian's 23.x in /usr/include. find_path searches the same prefixes in the
    # same order, so it locates the header the compiler will actually use.
    find_path(FLATBUFFERS_BASE_INCLUDE flatbuffers/base.h REQUIRED)
    file(STRINGS "${FLATBUFFERS_BASE_INCLUDE}/flatbuffers/base.h" fb_version_lines
         REGEX "#define FLATBUFFERS_VERSION_(MAJOR|MINOR|REVISION) ")
    set(header_version "")
    foreach(part MAJOR MINOR REVISION)
        string(REGEX MATCH "FLATBUFFERS_VERSION_${part} ([0-9]+)" _ "${fb_version_lines}")
        list(APPEND header_version "${CMAKE_MATCH_1}")
    endforeach()
    list(JOIN header_version "." header_version)
    execute_process(COMMAND "${FLATC_EXECUTABLE}" --version OUTPUT_VARIABLE flatc_version_out)
    string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" flatc_version "${flatc_version_out}")
    if(NOT flatc_version STREQUAL header_version)
        message(FATAL_ERROR
            "flatc ${flatc_version} (${FLATC_EXECUTABLE}) does not match the FlatBuffers headers "
            "the compiler will use: ${header_version} (${FLATBUFFERS_BASE_INCLUDE}). Install a "
            "flatc of version ${header_version}, or point -DFLATC_EXECUTABLE at one. See "
            "docs/decisions.md, Phase 3.")
    endif()

    file(GLOB fbs_files CONFIGURE_DEPENDS "${schema_dir}/*.fbs")
    set(gen_root "${CMAKE_CURRENT_BINARY_DIR}/generated")
    set(out_dir "${gen_root}/ar_drive_assist/common/schemas")

    set(outputs "")
    foreach(fbs IN LISTS fbs_files)
        get_filename_component(stem "${fbs}" NAME_WE)
        set(out "${out_dir}/${stem}_generated.h")
        add_custom_command(
            OUTPUT "${out}"
            COMMAND "${FLATC_EXECUTABLE}" --cpp --gen-object-api -o "${out_dir}" "${fbs}"
            DEPENDS "${fbs}"
            COMMENT "flatc: ${stem}.fbs"
            VERBATIM)
        list(APPEND outputs "${out}")
    endforeach()

    # An INTERFACE library that lists the generated headers as sources becomes a real build target
    # (CMake >= 3.19). Anything linking it therefore waits for flatc to finish.
    add_library(${name} INTERFACE ${outputs})
    target_include_directories(${name} INTERFACE "${gen_root}")
endfunction()
