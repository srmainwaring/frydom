# Exports the GeographicLib target

set(MPFR_LIBRARIES "")

include(FetchContent)

FetchContent_Declare(GeographicLib
        GIT_REPOSITORY ${geographiclib_URL}
        GIT_TAG ${geographiclib_TAG}
#        PATCH_COMMAND patch < "${PROJECT_SOURCE_DIR}/cmake/patches/${geographiclib_PATCH}"
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
#set(MagneticModel_URL "https://www.ngdc.noaa.gov/geomag/EMM/data/geomag/EMM2017_Sph_Linux.zip")
FetchContent_Declare(MagneticModel
        URL ${MagneticModel_URL}
        )
FetchContent_GetProperties(MagneticModel)

## FIXME : erreur de chargement de magnetic model a fixer
if (NOT magneticmodel_POPULATED)
    message(STATUS "******* FETCHING magnetic_model dependency from ${PROJECT_NAME} (requested version: ${MagneticModel_URL}) *******")
    FetchContent_Populate(MagneticModel)
    message(STATUS "Magnetic model downloaded: ${magneticmodel_SOURCE_DIR}")
#    set(GEOGRAPHICLIB_MAGNETIC_PATH "${magneticmodel_SOURCE_DIR}" CACHE STRING "")
    target_compile_definitions(GeographicLib INTERFACE -DGEOGRAPHICLIB_MAGNETIC_PATH="${magneticmodel_SOURCE_DIR}")
    # FIXME: ne peut pas fonctionner car dans ce cas le path du modele est code en dur dans l'exe...
endif ()
