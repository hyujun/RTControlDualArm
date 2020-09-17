###
# Set minimum version of CMake. Since command 'project' use
# VERSION sub-option we need at least 3.0.
# Note: If you use 2.6 or 2.4, God kills a kitten. Seriously.
cmake_minimum_required(VERSION 2.8)

####
# Set variables:
#   * PROJECT_NAME
#   * PROJECT_VERSION
project(RTBionicArm_m)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(FindPkgConfig)
include(${PROJECT_SOURCE_DIR}/modules/CMakeFunctions.cmake)

cmake_policy(SET CMP0022 NEW)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++${CMAKE_CXX_STANDARD} -O3" )
message(STATUS "C++${CMAKE_CXX_STANDARD} support has been enabled by default.")
set(CMAKE_MODULE_PATH  ${CMAKE_CURRENT_SOURCE_DIR}/modules)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})

set(PROJ_INSTALL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/bin/inc)
set(PROJ_INSTALL_BIN_DIR ${PROJECT_SOURCE_DIR}/bin)
set(PROJ_INSTALL_LIB_DIR ${PROJECT_SOURCE_DIR}/lib)

set(CTRL_HEADERS_DIR ${PROJECT_SOURCE_DIR}/Control)
set(ECAT_HEADERS_DIR ${PROJECT_SOURCE_DIR}/EcatSystem)
set(KDL_HEADERS_DIR ${PROJECT_SOURCE_DIR}/KDL)
set(NET_HEADERS_DIR ${PROJECT_SOURCE_DIR}/Network)

#include_directories(/usr/include)
#include_directories(/usr/include/linux)
#include_directories(/usr/include/c++/4.8.4)

include_directories(${PROJ_INSTALL_INCLUDE_DIR})
include_directories(
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
        devMouseObject.cpp
)
target_link_libraries(${PROJECT_NAME}
        sub::CTRL
        sub::ECAT
        sub::KDL
        sub::NETWORK
)

#find_package(Eigen3 CONFIG)
set( EIGEN3_INCLUDE_DIR /usr/include/eigen3)
IF( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
include_directories ( ${EIGEN3_INCLUDE_DIR} )
message(STATUS "Eigen3 includes : ${EIGEN3_INCLUDE_DIR}" )

find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif(OpenMP_CXX_FOUND)

find_package(Xenomai REQUIRED)
if(Xenomai_FOUND)
    message(STATUS "Xenomai Native includes : ${Xenomai_NATIVE_INCLUDE_DIRS}" )
    message(STATUS "Xenomai Native definitions: ${Xenomai_NATIVE_DEFINITIONS}")
    message(STATUS "Xenomai Native LIBRARY_DIRS : ${Xenomai_NATIVE_LIBRARY_DIRS}")
    message(STATUS "Xenomai Native LIBRARIES : ${Xenomai_NATIVE_LIBRARIES}")
    message(STATUS "Xenomai Native LDFLAGS : ${Xenomai_NATIVE_LDFLAGS}")
    message(STATUS "Xenomai Native DL_FLAGS : ${Xenomai_NATIVE_DL_FLAGS}")

    include_directories(${Xenomai_NATIVE_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${Xenomai_NATIVE_LIBRARY_DIRS} ${Xenomai_NATIVE_LIBRARIES})
#    set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS ${Xenomai_NATIVE_LDFLAGS})
    target_compile_definitions(${PROJECT_NAME} PUBLIC ${Xenomai_NATIVE_DEFINITIONS})
endif(Xenomai_FOUND)

find_package(Etherlab REQUIRED)
if(Etherlab_FOUND)
    message(STATUS "Etherlab_INC_DIRS = ${Etherlab_INCLUDE_DIR}")
    message(STATUS "Etherlab_LIB_DIRS = ${Etherlab_LIBRARIES}")
    include_directories(${Etherlab_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${Etherlab_LIBRARIES})
endif(Etherlab_FOUND)

find_package(NRMKHW REQUIRED)
if(NRMKHW_FOUND)
    message(STATUS "NRMKHW_INC_DIRS = ${NRMKHW_INCLUDE_DIR}")
    message(STATUS "NRMKHW_LIB_DIRS = ${NRMKHW_LIBRARIES}")
    include_directories(${NRMKHW_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${NRMKHW_LIBRARIES})
endif(NRMKHW_FOUND)

find_package(NRMKhelper REQUIRED)
if(NRMKhelper_FOUND)
    message(STATUS "NRMKhelper_INC_DIRS = ${NRMKhelper_INCLUDE_DIR}")
    message(STATUS "NRMKhelper_LIB_DIRS = ${NRMKhelper_LIBRARIES}")
    include_directories(${NRMKhelper_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${NRMKhelper_LIBRARIES})
endif(NRMKhelper_FOUND)

find_package(Poco REQUIRED COMPONENTS Foundation Util Net)
if (Poco_FOUND)
    MESSAGE(STATUS "Poco_INCLUDE_DIRS = ${Poco_INCLUDE_DIRS}")
    MESSAGE(STATUS "Poco_LIBRARIES = ${Poco_LIBRARIES}")
    include_directories(${Poco_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${Poco_LIBRARIES})
endif ()
