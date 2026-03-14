cmake_minimum_required(VERSION 3.8)

project(RTControlDualArm VERSION 2.0.0 LANGUAGES C CXX)

# ---------------------------------------------------------------------------
# Default build type
# ---------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Build type not set – defaulting to Release")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose: Debug Release RelWithDebInfo Profile" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo Profile)
endif()

# ---------------------------------------------------------------------------
# Module path
# ---------------------------------------------------------------------------
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(FindPkgConfig)
include(cmake/CMakeFunctions.cmake)

# ---------------------------------------------------------------------------
# Install directories (in-source layout: bin/ lib/ bin/inc/)
# ---------------------------------------------------------------------------
set(CMAKE_INSTALL_PREFIX     ${PROJECT_SOURCE_DIR})
set(PROJ_INSTALL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/bin/inc)
set(PROJ_INSTALL_BIN_DIR     ${PROJECT_SOURCE_DIR}/bin)
set(PROJ_INSTALL_LIB_DIR     ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_DEBUG_POSTFIX      "d")

# ---------------------------------------------------------------------------
# C++ standard  (C++17 required for inline variables, constexpr, structured bindings)
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Minimum compiler check
execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} -dumpfullversion -dumpversion
    OUTPUT_VARIABLE GCC_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
if(GCC_VERSION VERSION_LESS 5.3.1)
    message(FATAL_ERROR
        "g++ ${GCC_VERSION} found; ${PROJECT_NAME} requires g++ >= 5.3.1.")
endif()

# ---------------------------------------------------------------------------
# Compiler flags
# ---------------------------------------------------------------------------
# Common flags for all build types
add_compile_options(-Wall -Wextra -fPIC)

set(CMAKE_CXX_FLAGS_RELEASE        "-O2 -DNDEBUG -march=native -funroll-loops")
set(CMAKE_CXX_FLAGS_DEBUG
    "-g -fno-omit-frame-pointer -fno-inline-functions \
-fno-inline-functions-called-once -fno-optimize-sibling-calls")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_PROFILE        "${CMAKE_CXX_FLAGS_DEBUG} -pg")

# Catch undefined symbols at shared-library link time
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")

# ---------------------------------------------------------------------------
# Find Xenomai early (before targets that use it)
# ---------------------------------------------------------------------------
find_package(Xenomai 2.6.0 REQUIRED NATIVE)
message(STATUS "Xenomai found. Skins: ${Xenomai_SKINS}")

# ---------------------------------------------------------------------------
# Sub-libraries
# ---------------------------------------------------------------------------
add_subdirectory(src/control)
add_subdirectory(src/ecat)
add_subdirectory(src/kdl)
add_subdirectory(src/net)

# ---------------------------------------------------------------------------
# Main executable
# ---------------------------------------------------------------------------
add_executable(${PROJECT_NAME} src/main.cpp)

# Project-wide include root (src/ files use #include "ecat/...", "kdl/...", etc.)
target_include_directories(${PROJECT_NAME} PRIVATE include)

target_link_libraries(${PROJECT_NAME} PRIVATE
    sub::CTRL
    sub::ECAT
    sub::KDL
    sub::NETWORK
)

# ---------------------------------------------------------------------------
# Xenomai integration
# set_target_properties(COMPILE_FLAGS) in a loop would overwrite on each
# iteration.  Use target_compile_options / target_link_libraries instead.
# ---------------------------------------------------------------------------
if(Xenomai_FOUND)
    target_include_directories(${PROJECT_NAME} PRIVATE ${KERNELHEADERS_DIR})
    foreach(skin ${Xenomai_FOUND_COMP})
        target_include_directories(${PROJECT_NAME} PRIVATE
            ${Xenomai_${skin}_INCLUDE_DIRS})
        target_compile_options(${PROJECT_NAME} PRIVATE
            ${Xenomai_${skin}_DEFINITIONS})
        # DL_FLAGS are raw linker flags (e.g. -Wl,--wrap=...), not libraries
        target_link_libraries(${PROJECT_NAME} PRIVATE
            ${Xenomai_${skin}_DL_FLAGS})
    endforeach()
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin)

# ---------------------------------------------------------------------------
# Build summary (always shown – useful when diagnosing CI failures)
# ---------------------------------------------------------------------------
string(TOUPPER "${CMAKE_BUILD_TYPE}" _BUILD_TYPE_UPPER)
message(STATUS "")
message(STATUS "[ ${PROJECT_NAME} ${PROJECT_VERSION} – build summary ]")
message(STATUS "  Generator  : ${CMAKE_GENERATOR}")
message(STATUS "  Compiler   : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  C++ std    : ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Install    : ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  Flags      : ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${_BUILD_TYPE_UPPER}}")
message(STATUS "")
