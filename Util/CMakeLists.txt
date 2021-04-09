cmake_minimum_required(VERSION 3.8)
project(Util LANGUAGES CXX)

set(CMAKE_DEBUG_POSTFIX "d")
file(GLOB SOURCE_FILES "*.cpp" "*.c")

add_library(${PROJECT_NAME} SHARED STATIC ${SOURCE_FILES})

add_library(sub::Util ALIAS ${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME} PUBLIC ${PRJECT_SOURCE_DIR})
