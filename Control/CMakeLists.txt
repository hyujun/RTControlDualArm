cmake_minimum_required(VERSION 2.8)
project(Control )

add_library(${PROJECT_NAME} SHARED STATIC
        Controller.cpp
        KistHand.cpp
        Motion.cpp
        Trajectory.cpp
        )
add_library(sub::CTRL ALIAS ${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})

find_package(Eigen 3.0.0)
IF( Eigen_FOUND )
    add_definitions( ${Eigen_DEFINITIONS} )
    include_directories ( ${Eigen_INCLUDE_DIRS} )
ELSE()
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN_INCLUDE_DIR to the include directory of your Eigen installation.")
ENDIF()
