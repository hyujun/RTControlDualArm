if(NRMKHW_FOUND)
    return()
endif()

set(NRMKhelper_HINTS
        ./modules
        #${NRMKHW_DIR}
        )

if(NOT NRMKhelper_ROOT_DIR)
    message(STATUS "Looking for NRMK helper install directory ...")
    find_path(NRMKhelper_ROOT_DIR
            NAMES helper/inc/NRMKHelper.h
            HINTS ${NRMKhelper_HINTS}

            )
    if(NOT NRMKhelper_ROOT_DIR)
        message(FATAL_ERROR "NRMK helper: Could not find NRMK helper install directory")
    else()
        message(STATUS "NRMK helper: NRMK helper install dirctory structure at ${NRMK helper_ROOT_DIR}")
        set(NRMKhelper_INSTALLED true)
    endif()
endif()

if(UNIX)
    set(NRMKhelper_INCLUDE_DIR ${NRMKhelper_ROOT_DIR}/inc CACHE PATH "the global include path for NRMK helper")

    find_path(NRMKhelper_LIBRARY_DIR
            NAMES libNRMKHelperi686.a
            HINTS ${NRMKhelper_ROOT_DIR}
            PATH_SUFFIXES
            lib
            )
    if(NOT NRMKhelper_LIBRARY_DIR)
        message(FATAL_ERROR "NRMK helper: Could not find NRMK helper library directory")
    else()
        list(APPEND NRMKhelper_LIBRARIES ${NRMKhelper_LIBRARY_DIR})
    endif()
endif()

set(NRMKHW_FOUND true)