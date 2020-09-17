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
        list(APPEND NRMKHW_LIBRARY_DIR ${NRMKHW_LIBRARY_DIR})
    endif()
endif()

list(APPEND components_nrmkhw
        ${NRMKHW_FIND_COMPONENTS}
        "NRMKhw_tp"
        )
list(REMOVE_DUPLICATES components_nrmkhw)

foreach(comp ${components_nrmkhw})
    if(NOT NRMKHW_${comp}_LIBRARY)
        find_library(
                NRMKHW_${comp}_LIBRARY
                NAMES lib${comp}.a lib${comp}.so lib${comp}.la
                HINTS ${NRMKHW_LIBRARY_DIR}
                #PATH_SUFFIXES
                #lib
        )
        if(NRMKHW_${comp}_LIBRARY)
            message(STATUS "Found ${comp}: ${NRMKHW_${comp}_LIBRARY}")
        endif()
    endif()

    if(NRMKHW_${comp}_LIBRARY)
        list(APPEND NRMKHW_LIBRARIES ${NRMKHW_${comp}_LIBRARY} )
        mark_as_advanced(NRMKHW_${comp}_LIBRARY)
    endif()

    # mark component as found or handle not finding it
    if(NRMKHW_${comp}_LIBRARY)
        set(NRMKHW_${comp}_FOUND TRUE)
    elseif(NOT NRMKHW_FIND_QUIETLY)
        message(FATAL_ERROR "Could not find NRMKHW component ${comp}!")
    endif()
endforeach()

if(DEFINED NRMKHW_LIBRARIES)
    set(NRMKHW_FOUND true)
endif()
message(STATUS "Found NRMKHW: ${NRMKHW_LIBRARIES}")