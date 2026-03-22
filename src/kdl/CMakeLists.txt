cmake_minimum_required(VERSION 3.8)
project(KDL LANGUAGES CXX)

set(CMAKE_DEBUG_POSTFIX "d")

file(GLOB SOURCE_FILES "*.cpp" "*.c")
# Headers are not compiled; do not list them in add_library sources.

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
add_library(sub::KDL ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# OpenMP (required for dynamics parallelisation)
find_package(OpenMP REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)

# Eigen3
find_package(Eigen3 3.0 REQUIRED NO_MODULE)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

# Build Optimizations for Real-Time Performance
target_compile_options(${PROJECT_NAME} PRIVATE 
    -O3 
    -march=native 
    -funroll-loops 
    # -ffast-math # Uncomment after verifying it doesn't break Lie group precision (e.g., LogSO3)
)
