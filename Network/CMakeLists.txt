cmake_minimum_required(VERSION 3.8)
project(Network LANGUAGES CXX)

set(CMAKE_DEBUG_POSTFIX "d")
file(GLOB SOURCE_FILES "*.cpp" "*.c" "*.h")

add_library(${PROJECT_NAME} SHARED STATIC
            ${SOURCE_FILES}
            )

add_library(sub::NETWORK ALIAS ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(${PROJECT_NAME}
        INTERFACE ${PROJECT_SOURCE_DIR})

find_package(Poco REQUIRED COMPONENTS Foundation Util Zip Net)
if (Poco_FOUND)
    MESSAGE(STATUS "Poco_INCLUDE_DIRS = ${Poco_INCLUDE_DIRS}")
    MESSAGE(STATUS "Poco_LIBRARIES = ${Poco_LIBRARIES}")
    target_include_directories(${PROJECT_NAME} PUBLIC ${Poco_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${Poco_LIBRARIES})
endif ()

install(TARGETS ${PROJECT_MANE}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)