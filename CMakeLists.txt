###
# Set minimum version of CMake. Since command 'project' use
# VERSION sub-option we need at least 3.0.
# Note: If you use 2.6 or 2.4, God kills a kitten. Seriously.
cmake_minimum_required(VERSION 3.5)

cmake_policy(SET CMP0022 NEW)
if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif()

# If no build type was specified, set it to Release.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug Release CACHE STRING
        "Choose the type of build, options are: None Debug Release."
        FORCE)
endif(NOT CMAKE_BUILD_TYPE)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(FindPkgConfig)
include(./modules/CMakeFunctions.cmake)

cmake_policy(SET CMP0022 NEW)

# THIS HAS TO COME BEFORE THE PROJECT LINE
set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++${CMAKE_CXX_STANDARD} -Wall" )
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g3 -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

message(STATUS "C++${CMAKE_CXX_STANDARD} support has been enabled by default.")
set(CMAKE_MODULE_PATH  ${CMAKE_CURRENT_SOURCE_DIR}/modules)

set(BUILD_SHARED_LIBS TRUE)

####
# Set variables:
#   * PROJECT_NAME
#   * PROJECT_VERSION
project(RTControlDualArm)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})
set(PROJ_INSTALL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/bin/inc)
set(PROJ_INSTALL_BIN_DIR ${PROJECT_SOURCE_DIR}/bin)
set(PROJ_INSTALL_LIB_DIR ${PROJECT_SOURCE_DIR}/lib)

set(CTRL_HEADERS_DIR ${PROJECT_SOURCE_DIR}/Control)
set(ECAT_HEADERS_DIR ${PROJECT_SOURCE_DIR}/EcatSystem)
set(KDL_HEADERS_DIR ${PROJECT_SOURCE_DIR}/KDL)
set(NET_HEADERS_DIR ${PROJECT_SOURCE_DIR}/Network)

include_directories(${PROJ_INSTALL_INCLUDE_DIR})
include_directories(
        ${KERNELHEADERS_DIR}
        ${CTRL_HEADERS_DIR}
        ${ECAT_HEADERS_DIR}
        ${KDL_HEADERS_DIR}
        ${NET_HEADERS_DIR}
)

add_subdirectory(Control)
add_subdirectory(EcatSystem)
add_subdirectory(KDL)
add_subdirectory(Network)

add_executable(${PROJECT_NAME}
        RTClient.cpp
)
target_link_libraries(${PROJECT_NAME}
        sub::CTRL
        sub::ECAT
        sub::KDL
        sub::NETWORK
)

find_package(Xenomai 3.0.0 REQUIRED
             NATIVE
             #RTDM
             #POSIX
             )
message(STATUS "Xenomai found with theses skins: ${Xenomai_SKINS}")
if(Xenomai_FOUND)
    target_include_directories(${PROJECT_NAME} PUBLIC ${KERNELHEADERS_DIR})
    foreach(comp_lists ${Xenomai_FOUND_COMP})
        #message(STATUS "Xenomai ${comp_lists} includes : ${Xenomai_${comp_lists}_INCLUDE_DIRS}" )
        #message(STATUS "Xenomai ${comp_lists} definitions: ${Xenomai_${comp_lists}_DEFINITIONS}")
        #message(STATUS "Xenomai ${comp_lists} LIBRARY_DIRS : ${Xenomai_${comp_lists}_LIBRARY_DIRS} ")
        #message(STATUS "Xenomai ${comp_lists} LIBRARIES : ${Xenomai_${comp_lists}_LIBRARIES}")
        #message(STATUS "Xenomai ${comp_lists} LDFLAGS : ${Xenomai_${comp_lists}_LDFLAGS}")
        #message(STATUS "Xenomai ${comp_lists} DL_FLAGS : ${Xenomai_${comp_lists}_DL_FLAGS}")

        target_include_directories(${PROJECT_NAME} PUBLIC ${Xenomai_${comp_lists}_INCLUDE_DIRS})
        #target_link_libraries(${PROJECT_NAME} ${Xenomai_${comp_lists}_LIBRARY_DIRS} ${Xenomai_${comp_lists}_LIBRARIES})
        target_link_libraries(${PROJECT_NAME} ${Xenomai_${comp_lists}_DL_FLAGS})
        set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${Xenomai_${comp_lists}_DEFINITIONS})
    endforeach(comp_lists ${Xenomai_SKINS})
endif(Xenomai_FOUND)