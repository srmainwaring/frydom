
include(FetchContent)

FetchContent_Declare(json
        GIT_REPOSITORY ${json_URL}
        GIT_TAG ${json_TAG}
        )

FetchContent_GetProperties(json)
if (NOT json_POPULATED)
    message(STATUS "******* FETCHING json dependency from ${PROJECT_NAME} (requested version: ${json_TAG}) *******")
    FetchContent_Populate(json)

    # json BUILD OPTIONS
    set(JSON_BuildTests ON CACHE BOOL "")# CACHE BOOL "Build json tests")
    set(JSON_Install ON CACHE BOOL "")

    add_subdirectory(${json_SOURCE_DIR} ${json_BINARY_DIR})
endif ()
