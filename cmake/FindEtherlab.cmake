
include(FindPackageHandleStandardArgs)

set(Etherlab_HINTS
    /opt/etherlab
    /usr/etherlab
#    /usr/local
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
        list(APPEND Etherlab_LIBRARY_DIR ${Etherlab_LIBRARY_DIR})
    endif()
endif()

list(APPEND components_etherlab
        ${Etherlab_FIND_COMPONENTS}
        "ethercat"
        "ethercat_rtdm"
        )
list(REMOVE_DUPLICATES components_etherlab)

foreach(comp ${components_etherlab})
    if(NOT Etherlab_${comp}_LIBRARY)
        find_library(
                Etherlab_${comp}_LIBRARY
                NAMES lib${comp}.a lib${comp}.so lib${comp}.la
                HINTS ${Etherlab_LIBRARY_DIR}
                PATH_SUFFIXES
                    lib
        )
        if(Etherlab_${comp}_LIBRARY)
            message(STATUS "Found Etherlab ${comp}: ${Etherlab_${comp}_LIBRARY}")
        endif()
    endif()

    if(Etherlab_${comp}_LIBRARY)
        list(APPEND Etherlab_LIBRARIES ${Etherlab_${comp}_LIBRARY} )
        mark_as_advanced(Etherlab_${comp}_LIBRARY)
    endif()

    # mark component as found or handle not finding it
    if(Etherlab_${comp}_LIBRARY)
        set(Etherlab_${comp}_FOUND TRUE)
    elseif(NOT Etherlab_FIND_QUIETLY)
        message(FATAL_ERROR "Could not find Etherlab component ${comp}!")
    endif()
endforeach()

if(DEFINED Etherlab_LIBRARIES)
    set(Etherlab_FOUND true)
endif()
message(STATUS "Found Etherlab: ${Etherlab_LIBRARIES}")


