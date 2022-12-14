
include(FetchContent)

FetchContent_Declare(spdlog
        GIT_REPOSITORY ${spdlog_URL}
        GIT_TAG ${spdlog_TAG}
        )

FetchContent_GetProperties(spdlog)
if (NOT spdlog_POPULATED)
    message(STATUS "******* FETCHING spdlog dependency from ${PROJECT_NAME} (requested version: ${spdlog_TAG}) *******")
    FetchContent_Populate(spdlog)

    # spdlog BUILD OPTIONS
    set(SPDLOG_MASTER_PROJECT OFF CACHE BOOL "")
    set(SPDLOG_FMT_EXTERNAL ON CACHE BOOL "")


    add_subdirectory(${spdlog_SOURCE_DIR} ${spdlog_BINARY_DIR})
endif ()
