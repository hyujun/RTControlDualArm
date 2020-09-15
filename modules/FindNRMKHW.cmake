if(NRMKHW_FOUND)
    return()
endif()

set(NRMKHW_HINTS
        ./modules/hw
        #${NRMKHW_DIR}
)

if(NOT NRMKHW_ROOT_DIR)
    message(STATUS "Looking for NRMK HW install directory ...")
    find_path(NRMKHW_ROOT_DIR
            NAMES inc/NRMKhw_tp.h
            HINTS ${NRMKHW_HINTS}

    )
    if(NOT NRMKHW_ROOT_DIR)
        message(FATAL_ERROR "NRMK HW: Could not find NRMK HW install directory")
    else()
        message(STATUS "NRMK HW: NRMK HW install dirctory structure at ${NRMKHW_ROOT_DIR}")
        set(NRMKHW_INSTALLED true)
    endif()
endif()

if(UNIX)
    set(NRMKHW_INCLUDE_DIR ${NRMKHW_ROOT_DIR}/inc CACHE PATH "the global include path for NRMK HW")

    find_path(NRMKHW_LIBRARY_DIR
            NAMES libNRMKhw_tp.a
            HINTS ${NRMKHW_ROOT_DIR}
            PATH_SUFFIXES
            lib
            )
    if(NOT NRMKHW_LIBRARY_DIR)
        message(FATAL_ERROR "NRMK HW: Could not find NRMK HW library directory")
    else()
        list(APPEND NRMKHW_LIBRARIES ${NRMKHW_LIBRARY_DIR})
    endif()
endif()

set(NRMKHW_FOUND true)