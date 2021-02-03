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

find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif(OpenMP_CXX_FOUND)

find_package(Eigen 3.0.0)
IF( Eigen_FOUND )
    add_definitions( ${Eigen_DEFINITIONS} )
    target_include_directories (${PROJECT_NAME} PUBLIC ${Eigen_INCLUDE_DIRS} )
ELSE()
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN_INCLUDE_DIR to the include directory of your Eigen installation.")
ENDIF()
