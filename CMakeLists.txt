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

option(BUILD_SHARED_LIBS "" ON) # TODO: affiner
message(STATUS "BUILD_SHARED_LIBS set to ${BUILD_SHARED_LIBS} in frydom")

option(frydom_build_tests "Activate build tests" ON)
option(frydom_build_unit_test "Build unit tests" ON)
option(frydom_build_demo "Build demo tests" ON)
option(frydom_build_validation "Build demo tests" ON)
option(frydom_build_dev_test "Build dev test" ON)
option(frydom_download_demo_data "Downloading demo data" ON)
set(frydom_data_version "1.1.5" CACHE STRING "Data version")

option(frydom_build_tools "Build the standalone tools" ON)

option(frydom_use_irrlicht "Use Irrlicht" ON)
option(frydom_build_on_LIGER "For building on LIGER supercomputer" OFF)

option(BUILD_BOOST "Build boost instead of finding it on the system" ON)

if (frydom_build_on_LIGER)
    set(frydom_use_irrlicht FALSE CACHE BOOL "" FORCE)
endif ()


# Access to local cmake scripts
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

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


if (${frydom_download_demo_data})
    include(${CMAKE_CURRENT_SOURCE_DIR}/data/DownloadData.cmake)
    DownloadData()
endif ()


message(STATUS "BUILD_SHARED_LIBS set to ${BUILD_SHARED_LIBS} in frydom")

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
# Adding tests
#=============================================================================
if (${frydom_build_tests})
    enable_testing()
    include(Add_googletest)

    add_subdirectory(tests)
endif ()


#=============================================================================
# Adding tools
#=============================================================================
if (${frydom_build_tools})
    include(add_cxxopts)
    add_subdirectory(tools)
endif ()
