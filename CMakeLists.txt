# ============================================================================
# Unify_Link Library - CMake Configuration
# ============================================================================
cmake_minimum_required(VERSION 3.16)

project(unify_link
    VERSION 1.0.0
    DESCRIPTION "Unified communication protocol library for embedded systems"
    LANGUAGES CXX
)

# ============================================================================
# Build Options
# ============================================================================
option(UNIFY_LINK_BUILD_TESTS "Build unit tests" ON)
option(UNIFY_LINK_BUILD_EXAMPLES "Build examples" ON)
option(UNIFY_LINK_ENABLE_COVERAGE "Enable code coverage" OFF)
option(UNIFY_LINK_INSTALL "Generate install target" OFF)
option(UNIFY_LINK_BUILD_PYTHON "Build Python bindings" ON)
option(UNIFY_LINK_USE_THREADS "Link with system thread library when available" ON)

# ============================================================================
# C++ Standard Configuration
# ============================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for IDE integration
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================================
# Compiler Warnings Configuration
# ============================================================================
if(MSVC)
    add_compile_options(/W4 /WX- /utf-8)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# Header-Only Library Target
# ============================================================================
add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/component>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/component>
)

target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)

# Thread support (optional, for platforms that provide a thread library)
if(UNIFY_LINK_USE_THREADS)
    find_package(Threads QUIET)

    if(TARGET Threads::Threads)
        target_link_libraries(${PROJECT_NAME} INTERFACE Threads::Threads)
    else()
        message(WARNING "Threads library not found; continuing without thread linkage.")
    endif()
endif()

# ============================================================================
# Python bindings (pybind11)
# ============================================================================
if(UNIFY_LINK_BUILD_PYTHON)
    # Try to find pybind11 first (installed or via scikit-build-core)
    find_package(pybind11 CONFIG QUIET)

    if(NOT pybind11_FOUND)
        # Fall back to FetchContent if not found
        include(FetchContent)
        FetchContent_Declare(
            pybind11
            GIT_REPOSITORY https://github.com/pybind/pybind11.git
            GIT_TAG v2.12.0
        )
        FetchContent_MakeAvailable(pybind11)
    endif()

    pybind11_add_module(unify_link_py
        python/unify_link_wrap_py.cpp
    )

    # Rename output to unify_link.xxx (e.g., unify_link.so or unify_link.pyd)
    set_target_properties(unify_link_py PROPERTIES
        OUTPUT_NAME "unify_link"
    )

    target_link_libraries(unify_link_py PRIVATE ${PROJECT_NAME})

    # Building via scikit-build-core for pip
    install(TARGETS unify_link_py DESTINATION unify_link)
    install(FILES python/unify_link/__init__.py DESTINATION unify_link)

    # Development/standalone builds
    set_target_properties(unify_link_py PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python
    )

    # Copy output files to 'example/python'
    add_custom_command(TARGET unify_link_py POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:unify_link_py>
        ${CMAKE_CURRENT_SOURCE_DIR}/example/python
    )
endif()

# ============================================================================
# Code Coverage Configuration
# ============================================================================
if(UNIFY_LINK_ENABLE_COVERAGE AND NOT MSVC)
    add_compile_options(--coverage -O0 -g)
    add_link_options(--coverage)
endif()

# ============================================================================
# Unit Tests
# ============================================================================
if(UNIFY_LINK_BUILD_TESTS)
    enable_testing()

    # Fetch GoogleTest
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )

    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    include(GoogleTest)

    # Auto-register tests from test/*.cpp
    file(GLOB test_sources CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp
    )

    function(unify_link_add_test test_source)
        get_filename_component(test_name ${test_source} NAME_WE)

        add_executable(${test_name} ${test_source})
        target_link_libraries(${test_name} PRIVATE ${PROJECT_NAME})

        if(TARGET Threads::Threads)
            target_link_libraries(${test_name} PRIVATE Threads::Threads)
        endif()

        target_link_libraries(${test_name} PRIVATE GTest::gtest_main)
        gtest_discover_tests(${test_name})
    endfunction()

    foreach(test_source IN LISTS test_sources)
        unify_link_add_test(${test_source})
    endforeach()
endif()

# ============================================================================
# Examples
# ============================================================================
if(UNIFY_LINK_BUILD_EXAMPLES)
    add_executable(example_basic
        example/test.cpp
    )
    target_link_libraries(example_basic PRIVATE
        ${PROJECT_NAME}
    )
endif()

# ============================================================================
# Installation
# ============================================================================
if(UNIFY_LINK_INSTALL)
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)

    # Install headers
    install(FILES
        unify_link.hpp
        unify_link_def.h
        CRC16.hpp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    install(FILES
        component/motor_link.hpp
        component/encoder_link.hpp
        component/update_Link.hpp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/component
    )

    # Install targets
    install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    # Generate and install CMake config files
    install(EXPORT ${PROJECT_NAME}Targets
        FILE ${PROJECT_NAME}Targets.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    )

    configure_package_config_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    )

    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )

    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    )
endif()

# ============================================================================
# Summary
# ============================================================================
message(STATUS "")
message(STATUS "=== Unify_Link Configuration Summary ===")
message(STATUS "  Version:          ${PROJECT_VERSION}")
message(STATUS "  C++ Standard:     ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Tests:      ${UNIFY_LINK_BUILD_TESTS}")
message(STATUS "  Build Examples:   ${UNIFY_LINK_BUILD_EXAMPLES}")
message(STATUS "  Build Python:     ${UNIFY_LINK_BUILD_PYTHON}")
message(STATUS "  Enable Coverage:  ${UNIFY_LINK_ENABLE_COVERAGE}")
message(STATUS "  Install:          ${UNIFY_LINK_INSTALL}")
message(STATUS "=========================================")
message(STATUS "")
