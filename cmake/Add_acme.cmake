
include(FetchContent)

FetchContent_Declare(acme
        GIT_REPOSITORY ${acme_URL}
        GIT_TAG ${acme_TAG}
        )

FetchContent_GetProperties(acme)
if (NOT acme_POPULATED)
    message(STATUS "******* FETCHING acme dependency from ${PROJECT_NAME} (requested version: ${acme_TAG}) *******")
    FetchContent_Populate(acme)

    # hdb5_io BUILD OPTIONS
    set(ACME_BUILD_TESTS OFF CACHE BOOL "")
    set(ACME_BUILD_DEV_TESTS OFF CACHE BOOL "")
    set(ACME_BUILD_UNIT_TESTS OFF CACHE BOOL "")

    add_subdirectory(${acme_SOURCE_DIR} ${acme_BINARY_DIR})
endif ()
