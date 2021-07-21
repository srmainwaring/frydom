#=============================================================================
#
# CMake configuration file for the FRyDoM project
#
#=============================================================================
cmake_minimum_required(VERSION 3.13)

project(frydom)
set(CMAKE_CXX_STANDARD 17)

#-----------------------------------------------------------------------------
# Prevent in-source builds
#-----------------------------------------------------------------------------
if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source build is not possible and not recommended. Choose an empty directory for build output.")
endif (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

#-----------------------------------------------------------------------------
# FRyDoM options
#-----------------------------------------------------------------------------
cmake_policy(SET CMP0077 NEW)

option(BUILD_SHARED_LIBS "Shared lib version" ON)

option(FRYDOM_BUILD_TESTS "Activate build tests" ON)
option(FRYDOM_BUILD_UNIT_TESTS "Build unit tests" ON)
option(FRYDOM_BUILD_DEMOS "Build demo tests" ON)
option(FRYDOM_BUILD_BENCHES "Build benches tests" ON)
option(FRYDOM_BUILD_DEV_TESTS "Build dev test" ON)
option(FRYDOM_BUILD_TOOLS "Build the standalone tools" ON)
option(FRYDOM_USE_IRRLICHT "Use Irrlicht" ON)
option(FRYDOM_BUILD_ON_LIGER "For building on LIGER supercomputer" OFF)
option(BUILD_BOOST "Build boost instead of finding it on the system" ON)

option(FRYDOM_DOWNLOAD_DEMOS_DATA "Downloading demo data" ON)
set(FRYDOM_DEMOS_DATA_VERSION "1.1.6" CACHE STRING "Data version")

if (FRYDOM_BUILD_ON_LIGER)
    set(FRYDOM_USE_IRRLICHT FALSE CACHE BOOL "" FORCE)
endif ()

#=============================================================================
# Retrieving the current Git revision
#=============================================================================
set(PRE_CONFIGURE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/frydom/version.cpp.in CACHE FILEPATH "" FORCE)
set(POST_CONFIGURE_FILE ${CMAKE_CURRENT_BINARY_DIR}/src/frydom/version.cpp CACHE FILEPATH "" FORCE)
set(GIT_WORKING_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE FILEPATH "" FORCE)
set(GIT_STATE_FILE ${CMAKE_CURRENT_BINARY_DIR}/git-state-hash CACHE FILEPATH "" FORCE)
include(cmake/git_watcher.cmake)

if (NOT EXISTS ${POST_CONFIGURE_FILE})
    configure_file(${PRE_CONFIGURE_FILE} ${POST_CONFIGURE_FILE})
endif ()

# =============================================================================
# FRyDoM DEPENDENCIES
# =============================================================================
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/URL.conf.cmake)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include(Add_mathutils)
include(Add_Boost)
include(Add_chrono)
include(Add_hdb5_io)
include(Add_GeographicLib)
include(Add_cppfs)
include(Add_TimeZone)
include(Add_hermes)
include(Add_json)
include(Add_spdlog)

#=============================================================================
# Adding the FRyDoM project sources
#=============================================================================
add_subdirectory(src/frydom)

#=============================================================================
# Downloading data for demos from Amazon AWS
#=============================================================================
if (${FRYDOM_DOWNLOAD_DEMOS_DATA})
    include(${CMAKE_CURRENT_SOURCE_DIR}/data/DownloadData.cmake)
    DownloadData()
endif ()

#=============================================================================
# Adding tests
#=============================================================================
if (${FRYDOM_BUILD_TESTS})
    enable_testing()
    include(Add_googletest)
    add_subdirectory(tests)
endif ()

#=============================================================================
# Adding tools
#=============================================================================
if (${FRYDOM_BUILD_TOOLS})
    include(add_cxxopts)
    add_subdirectory(tools)
endif ()
