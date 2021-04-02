
include(FetchContent)

FetchContent_Declare(hdb5io
        GIT_REPOSITORY ${hdb5io_URL}
        GIT_TAG ${hdb5io_TAG}
        )

FetchContent_GetProperties(hdb5io)
if (NOT hdb5io_POPULATED)
    message(STATUS "******* FETCHING hdb5_io dependency from ${PROJECT_NAME} (requested version: ${hdb5io_TAG}) *******")
    FetchContent_Populate(hdb5io)

    # Hdb5io BUILD OPTIONS
    set(HDB5IO_BUILD_TESTS OFF CACHE BOOL "")
    set(HDB5IO_USE_VTK OFF CACHE BOOL "")

    add_subdirectory(${hdb5io_SOURCE_DIR} ${hdb5io_BINARY_DIR})
endif ()
