# Exports the GeographicLib target

set(MPFR_LIBRARIES "")

include(FetchContent)

FetchContent_Declare(GeographicLib
        GIT_REPOSITORY ${geographiclib_URL}
        GIT_TAG ${geographiclib_TAG}
        PATCH_COMMAND patch < "${PROJECT_SOURCE_DIR}/cmake/patches/${geographiclib_PATCH}"
        )

FetchContent_GetProperties(GeographicLib)
if (NOT geographiclib_POPULATED)
    message(STATUS "******* FETCHING geographiclib dependency from ${PROJECT_NAME} (requested version: ${geographiclib_TAG}) *******")
    FetchContent_Populate(GeographicLib)

    # GeographicLib BUILD OPTIONS
    set(GEOGRAPHICLIB_LIB_TYPE SHARED CACHE BOOL "")
    set(GEOGRAPHICLIB_DOCUMENTATION OFF CACHE BOOL "")

    add_subdirectory(${geographiclib_SOURCE_DIR} ${geographiclib_BINARY_DIR})

endif ()

set(MagneticModel_URL https://sourceforge.net/projects/geographiclib/files/magnetic-distrib/emm2017.tar.bz2)
FetchContent_Declare(MagneticModel
        URL ${MagneticModel_URL}
        )
FetchContent_GetProperties(MagneticModel)

## FIXME : erreur de chargement de magnetic model a fixer
if (NOT magneticmodel_POPULATED)
    message(STATUS "******* FETCHING magnetic_model dependency from ${PROJECT_NAME} (requested version: ${MagneticModel_URL}) *******")
    TARGET_COMPILE_OPTIONS(GeographicLib PUBLIC -DGEOGRAPHICLIB_MAGNETIC_PATH="${magneticmodel_SOURCE_DIR}")
endif ()
