include(FetchContent)

FetchContent_Declare(cppfs
        GIT_REPOSITORY ${cppfs_URL}
        GIT_TAG ${cppfs_TAG}
        )

FetchContent_GetProperties(cppfs)
if (NOT cppfs_POPULATED)
    message(STATUS "******* FETCHING cppfs dependency from ${PROJECT_NAME} (requested version: ${cppfs_TAG}) *******")
    FetchContent_Populate(cppfs)

    # CPPFS BUILD OPTIONS
    set(OPTION_BUILD_TESTS OFF CACHE BOOL "")
    set(OPTION_BUILD_DOCS OFF CACHE BOOL "")
    set(OPTION_BUILD_EXAMPLES OFF CACHE BOOL "")
    set(OPTION_BUILD_SSH_BACKEND OFF CACHE BOOL "")

    add_subdirectory(${cppfs_SOURCE_DIR} ${cppfs_BINARY_DIR})

endif ()
