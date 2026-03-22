cmake_minimum_required(VERSION 3.8)
project(EcatSystem LANGUAGES C CXX)

set(CMAKE_DEBUG_POSTFIX "d")

file(GLOB_RECURSE SOURCE_FILES "*.cpp" "*.c")

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
add_library(sub::ECAT ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

find_package(Etherlab REQUIRED)
if(Etherlab_FOUND)
    message(STATUS "Etherlab include : ${Etherlab_INCLUDE_DIR}")
    message(STATUS "Etherlab library : ${Etherlab_LIBRARIES}")
    target_include_directories(${PROJECT_NAME} PUBLIC ${Etherlab_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${Etherlab_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME}   # was PROJECT_MANE (typo) – now correct
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
