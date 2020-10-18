cmake_minimum_required(VERSION 2.8)
project(EcatSystem)

add_library(${PROJECT_NAME} SHARED STATIC
        Ecat_Master.cpp
        PDOConfig.c
        )
add_library(sub::ECAT ALIAS ${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
        PUBLIC ${PRJECT_SOURCE_DIR})

#find_package(Eigen3 CONFIG)
#set( EIGEN3_INCLUDE_DIR /usr/include/eigen3)
#IF( NOT EIGEN3_INCLUDE_DIR )
#    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
#ENDIF()
#include_directories ( ${EIGEN3_INCLUDE_DIR} )
#message(STATUS "Eigen3 includes : ${EIGEN3_INCLUDE_DIR}" )

find_package(Etherlab REQUIRED)
if(Etherlab_FOUND)
    message(STATUS "Etherlab_INC_DIRS = ${Etherlab_INCLUDE_DIR}")
    message(STATUS "Etherlab_LIB_DIRS = ${Etherlab_LIBRARIES}")
    include_directories(${Etherlab_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${Etherlab_LIBRARIES})
endif(Etherlab_FOUND)