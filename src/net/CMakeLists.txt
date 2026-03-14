cmake_minimum_required(VERSION 3.8)
project(Network LANGUAGES CXX)

set(CMAKE_DEBUG_POSTFIX "d")

# .h files are intentionally excluded from GLOB – headers are not sources.
file(GLOB SOURCE_FILES "*.cpp" "*.c")

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
add_library(sub::NETWORK ALIAS ${PROJECT_NAME})

# PUBLIC so consumers that link sub::NETWORK can also find these headers.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

find_package(Poco REQUIRED COMPONENTS Foundation Util Zip Net)
if(Poco_FOUND)
    message(STATUS "Poco include : ${Poco_INCLUDE_DIRS}")
    message(STATUS "Poco libs    : ${Poco_LIBRARIES}")
    target_include_directories(${PROJECT_NAME} PUBLIC ${Poco_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${Poco_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME}   # was PROJECT_MANE (typo) – now correct
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
