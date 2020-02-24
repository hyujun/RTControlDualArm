
include(FindPackageHandleStandardArgs)

set(Etherlab_HINTS
    /opt/etherlab
    /usr
    /usr/local
)

if(NOT Etherlab_ROOT_DIR)
    message(STATUS "Looking for Etherlab install directory ...")
    find_path(Etherlab_ROOT_DIR
        NAMES include/ecrt.h
        HINTS ${Etherlab_HINTS}
    )
    if(NOT Etherlab_ROOT_DIR)
        message(FATAL_ERROR "Etherlab: Could not find Etherlab install directory")
    else()
        message(STATUS "Etherlab: Etherlab install dirctory structure at ${Etherlab_ROOT_DIR}")
        set(Etherlab_INSTALLED true)
    endif()
endif()

if(UNIX)
    set(Etherlab_INCLUDE_DIR ${Etherlab_ROOT_DIR}/include CACHE PATH "the global include path for Etherlab")

    find_path(Etherlab_LIBRARY_DIR
        NAMES libethercat.a
        HINTS ${Etherlab_ROOT_DIR}
        PATH_SUFFIXES
            lib
    )
    if(NOT Etherlab_LIBRARY_DIR)
        message(FATAL_ERROR "Etherlab: Could not find Etherlab library directory")
    else()
        list(APPEND Etherlab_LIBRARIES ${Etherlab_LIBRARY_DIR})
    endif()
endif()

set(Etherlab_FOUND true)



