cmake_minimum_required(VERSION 2.8)
project(KDL)

add_library(${PROJECT_NAME} SHARED STATIC
        LieDynamics.cpp
        LieOperator.cpp
        PoEKinematics.cpp
        PropertyDefinition.cpp
        SerialManipulator.cpp
        )

add_library(sub::KDL ALIAS ${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
        PUBLIC ${PROJECT_SOURCE_DIR})

find_package(Eigen 3.0.0)
IF( NOT Eigen_FOUND )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN_INCLUDE_DIR to the include directory of your Eigen installation.")
ENDIF()
add_definitions(${Eigen_DEFINITIONS})
include_directories ( ${Eigen_INCLUDE_DIRS} )
message(STATUS "Eigen includes : ${Eigen_INCLUDE_DIRS}" )