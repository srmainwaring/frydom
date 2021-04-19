include(FetchContent)

FetchContent_Declare(TimeZone
        GIT_REPOSITORY ${timezone_URL}
        GIT_TAG ${timezone_TAG}
        )

FetchContent_GetProperties(TimeZone)
if (NOT timezone_POPULATED)
    message(STATUS "******* FETCHING timezone dependency from ${PROJECT_NAME} (requested version: ${timezone_TAG}) *******")
    FetchContent_Populate(TimeZone)

    # TimeZone BUILD OPTIONS
    if (frydom_build_on_LIGER)
        set(USE_SYSTEM_TZ_DB ON CACHE BOOL "Use the operating system's timezone database")
    else ()
        set(USE_SYSTEM_TZ_DB OFF CACHE BOOL "Use the operating system's timezone database")
    endif ()
    set(USE_TZ_DB_IN_DOT OFF CACHE BOOL "Save the timezone database in the current folder")
#    set(BUILD_SHARED_LIBS ON CACHE BOOL "Build a shared version of library" FORCE)
    set(ENABLE_DATE_TESTING OFF CACHE BOOL "Enable unit tests")

    add_subdirectory(${timezone_SOURCE_DIR} ${timezone_BINARY_DIR})
endif ()
