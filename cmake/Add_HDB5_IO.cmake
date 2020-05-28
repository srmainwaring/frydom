
find_package(hdb5io QUIET)

if (NOT hdb5io)
    include(FetchContent)

#    set(FETCHCONTENT_QUIET OFF)

    FetchContent_Declare(hdb5io
            GIT_REPOSITORY ${hdb5io_URL}
            GIT_TAG ${hdb5io_TAG}
            )

    FetchContent_GetProperties(hdb5io)
    if(NOT hdb5io_POPULATED)
        message(STATUS "Downloading, Configuring and Generating 'HDB5_IO' dependency")
        FetchContent_Populate(hdb5io)

        # Hdb5io BUILD OPTIONS
        set(HDB5IO_build_tests OFF CACHE BOOL "" FORCE)
#        set(HDB5IO_USE_VTK ON CACHE BOOL "" FORCE)

        add_subdirectory(${hdb5io_SOURCE_DIR} ${hdb5io_BINARY_DIR})
    else()
        message(STATUS "Hdb5io already populated")
    endif()
endif()

if (TARGET hdb5io)
    get_target_property(INC hdb5io INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "Found hdb5io : ${INC}")
else()
    message(STATUS "hdb5io target NOT FOUND")
endif()
