cmake_minimum_required(VERSION 3.8)
project(Control LANGUAGES CXX)

set(CMAKE_DEBUG_POSTFIX "d")

# Explicit source list is preferred, but GLOB is acceptable here because
# new source files are rare; re-run cmake manually if you add one.
file(GLOB SOURCE_FILES "*.cpp" "*.c")

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
add_library(sub::CTRL ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Eigen3
find_package(Eigen3 3.0 REQUIRED NO_MODULE)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

# Boost (filesystem + system)
find_package(Boost COMPONENTS filesystem system QUIET)
if(Boost_FOUND)
    message(STATUS "Boost version: ${Boost_VERSION}")
    target_link_libraries(${PROJECT_NAME} PUBLIC Boost::filesystem Boost::system)
else()
    message(WARNING "Boost not found; Control library may not link correctly.")
endif()
