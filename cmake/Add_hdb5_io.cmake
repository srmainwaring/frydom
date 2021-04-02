
include(FetchContent)

FetchContent_Declare(hdb5_io
        GIT_REPOSITORY ${hdb5_io_URL}
        GIT_TAG ${hdb5_io_TAG}
        )

FetchContent_GetProperties(hdb5_io)
if (NOT hdb5_io_POPULATED)
    message(STATUS "******* FETCHING hdb5_io dependency from ${PROJECT_NAME} (requested version: ${hdb5_io_TAG}) *******")
    FetchContent_Populate(hdb5_io)

    # hdb5_io BUILD OPTIONS
    set(hdb5_io_BUILD_TESTS OFF CACHE BOOL "")
    set(hdb5_io_USE_VTK OFF CACHE BOOL "")

    add_subdirectory(${hdb5_io_SOURCE_DIR} ${hdb5_io_BINARY_DIR})
endif ()
