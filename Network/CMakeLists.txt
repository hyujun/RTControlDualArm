cmake_minimum_required(VERSION 2.8)
project(Network)

set(SOURCE_FILES SocketHandler.h EcatDataSocket.h EcatControlSocket.h)

add_library(${PROJECT_NAME} SHARED STATIC
        ${SOURCE_FILES}
        )
add_library(sub::NETWORK ALIAS ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(${PROJECT_NAME}
        INTERFACE ${PROJECT_SOURCE_DIR})

find_package(Poco REQUIRED COMPONENTS Foundation Util Net)
if (Poco_FOUND)
    MESSAGE(STATUS "Poco_INCLUDE_DIRS = ${Poco_INCLUDE_DIRS}")
    MESSAGE(STATUS "Poco_LIBRARIES = ${Poco_LIBRARIES}")
    include_directories(${Poco_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${Poco_LIBRARIES})
endif ()