cmake_minimum_required(VERSION 3.9)
project(RTControlDualArm)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(FindPkgConfig)
include(${PROJECT_SOURCE_DIR}/modules/CMakeFunctions.cmake)

cmake_policy(SET CMP0022 NEW)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH  ${CMAKE_CURRENT_SOURCE_DIR}/modules)

add_executable(${PROJECT_NAME} RTClient.cpp)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

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
    set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS ${Xenomai_NATIVE_LDFLAGS})
    target_compile_definitions(${PROJECT_NAME} PUBLIC ${Xenomai_NATIVE_DEFINITIONS})
endif(Xenomai_FOUND)

find_package(Etherlab REQUIRED)
if(Etherlab_FOUND)
    message(STATUS "Etherlab_INC_DIRS = ${Etherlab_INCLUDE_DIR}")
    message(STATUS "Etherlab_LIB_DIRS = ${Etherlab_LIBRARIES}")
    include_directories(${Etherlab_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${Etherlab_LIBRARIES})
endif(Etherlab_FOUND)

find_package(Poco REQUIRED COMPONENTS Foundation Util Net XML JSON)
if (Poco_FOUND)
    MESSAGE(STATUS "Poco_INCLUDE_DIRS = ${Poco_INCLUDE_DIRS}")
    MESSAGE(STATUS "Poco_LIBRARIES = ${Poco_LIBRARIES}")
    include_directories(${Poco_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${Poco_LIBRARIES})
endif ()