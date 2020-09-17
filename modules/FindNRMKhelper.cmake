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
        message(STATUS "NRMK helper: NRMK helper install dirctory structure at ${NRMKhelper_ROOT_DIR}")
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
        list(APPEND NRMKhelper_LIBRARY_DIR ${NRMKhelper_LIBRARY_DIR})
    endif()
endif()

list(APPEND components
        ${NRMKhelper_FIND_COMPONENTS}
        "NRMKHelperi686"
        )
list(REMOVE_DUPLICATES components)

foreach(comp ${components})
    if(NOT NRMKhelper_${comp}_LIBRARY)
        find_library(
                NRMKhelper_${comp}_LIBRARY
                NAMES lib${comp}.a lib${comp}.so lib${comp}.la
                HINTS ${NRMKhelper_LIBRARY_DIR}
                PATH_SUFFIXES
                lib
        )
        if(NRMKhelper_${comp}_LIBRARY)
            message(STATUS "Found NRMKhelper ${comp}: ${NRMKhelper_${comp}_LIBRARY}")
        endif()
    endif()

    if(NRMKhelper_${comp}_LIBRARY)
        list(APPEND NRMKhelper_LIBRARIES ${NRMKhelper_${comp}_LIBRARY} )
        mark_as_advanced(NRMKhelper_${comp}_LIBRARY)
    endif()

    # mark component as found or handle not finding it
    if(NRMKhelper_${comp}_LIBRARY)
        set(NRMKhelper_${comp}_FOUND TRUE)
    elseif(NOT NRMKhelper_FIND_QUIETLY)
        message(FATAL_ERROR "Could not find NRMKhelper component ${comp}!")
    endif()
endforeach()

if(DEFINED NRMKhelper_LIBRARIES)
    set(NRMKhelper_FOUND true)
endif()
message(STATUS "Found NRMKhelper: ${NRMKhelper_LIBRARIES}")
