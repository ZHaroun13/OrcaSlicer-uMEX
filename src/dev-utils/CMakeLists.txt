
option(SLIC3R_ENC_CHECK "Verify encoding of source files" 1)

if (IS_CROSS_COMPILE)
    # Force disable due to cross compilation. This fact is already printed on cli for users
    set(SLIC3R_ENC_CHECK OFF CACHE BOOL "" FORCE)
endif ()

if (SLIC3R_ENC_CHECK)
    add_executable(encoding-check encoding-check.cpp)

    # A global no-op target which depends on all encodings checks,
    # and on which in turn all checked targets depend.
    # This is done to make encoding checks the first thing to be
    # performed before actually compiling any sources of the checked targets
    # to make the check fail as early as possible.
    add_custom_target(global-encoding-check
        ALL
        DEPENDS encoding-check
    )
endif()

if (ORCA_TOOLS)
    set(_DEV_DEFS -DBOOST_ALL_NO_LIB -DBOOST_USE_WINAPI_VERSION=0x602 -DBOOST_SYSTEM_USE_UTF8)

    # generate_system_cache: pre-generates per-vendor <vendor>.opc files under resources/profiles for CI bundling.
    add_executable(generate_system_cache generate_system_cache.cpp)
    target_link_libraries(generate_system_cache libslic3r boost_headeronly)
    target_compile_definitions(generate_system_cache PRIVATE ${_DEV_DEFS})

    # texture_unwrap_dump: reports the LSCM unwrap of a saved project's texture displacement layers,
    # chart by chart, so a defect can be reproduced from the project file instead of from a screenshot.
    add_executable(texture_unwrap_dump texture_unwrap_dump.cpp)
    target_link_libraries(texture_unwrap_dump libslic3r boost_headeronly nanosvg)
    target_compile_definitions(texture_unwrap_dump PRIVATE ${_DEV_DEFS})

    # diw_translate: runs the DIW / u-MEX G-code translation on an existing file (regression tests, diw/tests).
    add_executable(diw_translate diw_translate.cpp)
    target_link_libraries(diw_translate libslic3r boost_headeronly nlohmann_json)
    target_compile_definitions(diw_translate PRIVATE ${_DEV_DEFS})

    # diw_planner_tests: the Catch2 unit tests of the DIW powder planner (tests/libslic3r/test_diw_planner.cpp,
    # diw/PLANNER_DESIGN.md T9) as a stand-alone executable, so they build without BUILD_TESTS (which would
    # pull in the whole test suite). With BUILD_TESTS the same file is part of libslic3r_tests instead.
    if (NOT BUILD_TESTS)
        if (NOT TARGET Catch2WithMain)
            add_subdirectory(${CMAKE_SOURCE_DIR}/tests/catch2 ${CMAKE_BINARY_DIR}/diw_catch2 EXCLUDE_FROM_ALL)
            if (MSVC)
                target_compile_definitions(Catch2WithMain PRIVATE DO_NOT_USE_WMAIN)
            endif ()
        endif ()
        set(_DIW_TEST_DATA_DIR ${CMAKE_SOURCE_DIR}/tests/data)
        file(TO_NATIVE_PATH "${_DIW_TEST_DATA_DIR}" _DIW_TEST_DATA_DIR)
        add_executable(diw_planner_tests ${CMAKE_SOURCE_DIR}/tests/libslic3r/test_diw_planner.cpp
                                         ${CMAKE_SOURCE_DIR}/tests/libslic3r/test_diw_shadow.cpp)
        target_link_libraries(diw_planner_tests libslic3r boost_headeronly nlohmann_json Catch2::Catch2WithMain)
        target_compile_definitions(diw_planner_tests PRIVATE ${_DEV_DEFS} TEST_DATA_DIR=R"\(${_DIW_TEST_DATA_DIR}\)" CATCH_CONFIG_FAST_COMPILE)
        target_include_directories(diw_planner_tests PRIVATE ${CMAKE_SOURCE_DIR}/tests)
    endif ()

    # profile_include_dump: prints what included templates contribute to a vendor's presets,
    # to diff against the same tool built in BambuStudio. Built only on request.
    add_executable(profile_include_dump EXCLUDE_FROM_ALL profile_include_dump.cpp)
    target_link_libraries(profile_include_dump libslic3r boost_headeronly)
    target_compile_definitions(profile_include_dump PRIVATE ${_DEV_DEFS})

endif()

# Function that adds source file encoding check to a target
# using the above encoding-check binary

function(encoding_check TARGET)
    if (SLIC3R_ENC_CHECK)
        # Obtain target source files
        get_target_property(T_SOURCES ${TARGET} SOURCES)

        set(STAMP_FILE  "${CMAKE_CURRENT_BINARY_DIR}/encoding-check-${TARGET}.stamp")
        set(RSP_FILE    "${CMAKE_CURRENT_BINARY_DIR}/encoding-check-${TARGET}.rsp")

        # Resolve absolute paths
        set(ABS_SOURCES "")
        foreach(file ${T_SOURCES})
            if(IS_ABSOLUTE "${file}")
                list(APPEND ABS_SOURCES "${file}")
            else()
                list(APPEND ABS_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/${file}")
            endif()
        endforeach()

        # Write response file at configure time
        string(REPLACE ";" "\n" RSP_CONTENT "${ABS_SOURCES}")
        file(WRITE "${RSP_FILE}" "${RSP_CONTENT}")

        # Single process call via response file — no command line length limit
        add_custom_command(
            OUTPUT  "${STAMP_FILE}"
            DEPENDS encoding-check ${ABS_SOURCES}
            COMMENT "Checking source files encodings for target ${TARGET}"
            COMMAND $<TARGET_FILE:encoding-check> ${TARGET} "@${RSP_FILE}"
            COMMAND ${CMAKE_COMMAND} -E touch "${STAMP_FILE}"
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )

        add_custom_target(encoding-check-${TARGET} DEPENDS "${STAMP_FILE}")
        add_dependencies(global-encoding-check encoding-check-${TARGET})
        add_dependencies(${TARGET} global-encoding-check)
    endif()
endfunction()
