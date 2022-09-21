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

set(FR_C_FLAGS "${CMAKE_C_FLAGS}")
set(FR_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

option(BUILD_SHARED_LIBS "Shared lib version" ON)

option(FRYDOM_BUILD_TESTS "Activate build tests" ON)
option(FRYDOM_BUILD_UNIT_TESTS "Build unit tests" ON)
option(FRYDOM_BUILD_DEMOS "Build demo tests" ON)
option(FRYDOM_BUILD_BENCHES "Build benches tests" ON)
option(FRYDOM_BUILD_DEV_TESTS "Build dev test" ON)
option(FRYDOM_BUILD_TOOLS "Build the standalone tools" ON)
option(BUILD_ON_LIGER "For building on LIGER supercomputer" OFF)
if (BUILD_ON_LIGER)
    set(FRYDOM_USE_IRRLICHT FALSE CACHE BOOL "" FORCE)
endif ()
option(FRYDOM_USE_IRRLICHT "Use Irrlicht" ON)
message(STATUS BUILD_ON_LIGER: ${BUILD_ON_LIGER})
message(STATUS FRYDOM_USE_IRRLICHT: ${FRYDOM_USE_IRRLICHT})
option(BUILD_BOOST "Build boost instead of finding it on the system" ON)

option(FRYDOM_DOWNLOAD_DEMOS_DATA "Downloading demo data" ON)
set(FRYDOM_DEMOS_DATA_VERSION "1.1.7" CACHE STRING "Data version")

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

#-----------------------------------------------------------------------------
# Include cmake
#-----------------------------------------------------------------------------

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/URL.conf.cmake)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(MKL_INCLUDE_DIR "/usr/include/mkl/")

#-----------------------------------------------------------------------------
# SSE / AVX / FMA / NEON support
#-----------------------------------------------------------------------------

option(USE_SIMD "Enable use of SIMD if supported (SSE, AVX, NEON)" ON)

if(USE_SIMD)

    # Figure out SIMD support
    message(STATUS "Testing SIMD capabilities...")
    find_package(SIMD)

    # Set substitution variables for configuration file
    if(SIMD_SSE)
        set(FRYDOM_HAS_SSE "#define FRYDOM_HAS_SSE")
        set(FRYDOM_SSE_LEVEL "#define FRYDOM_SSE_LEVEL \"${SIMD_SSE}\"")

        # SSE up to and including 2.0 is supported on all 64-bit x86 systems
        set(FRYDOM_SSE_1_0 "#define FRYDOM_SSE_1_0")
        set(FRYDOM_SSE_2_0 "#define FRYDOM_SSE_2_0")

        if(${SIMD_SSE} VERSION_GREATER_EQUAL 3.0)
            set(FRYDOM_SSE_3_0 "#define FRYDOM_SSE_3_0")
        endif()
        if(${SIMD_SSE} VERSION_GREATER_EQUAL 4.1)
            set(FRYDOM_SSE_4_1 "#define FRYDOM_SSE_4_1")
        endif()
        if(${SIMD_SSE} VERSION_GREATER_EQUAL 4.2)
            set(FRYDOM_SSE_4_2 "#define FRYDOM_SSE_4_2")
        endif()
    endif()

    # Figure out AVX support

    # Set substitution variables for configuration file
    if(SIMD_AVX)
        set(FRYDOM_HAS_AVX "#define FRYDOM_HAS_AVX")
        set(FRYDOM_AVX_LEVEL "#define FRYDOM_AVX_LEVEL \"${SIMD_AVX}\"")
        if(${SIMD_AVX} VERSION_GREATER_EQUAL 1.0)
            set(FRYDOM_AVX_1_0 "#define FRYDOM_AVX_1_0")
        endif()
        if(${SIMD_AVX} VERSION_GREATER_EQUAL 2.0)
            set(FRYDOM_AVX_2_0 "#define FRYDOM_AVX_2_0")
        endif()
    endif()

    # Figure out FMA support
    set(ALLOW_FMA TRUE)
    if(MSVC)
        if(NOT CH_WHOLE_PROG_OPT)
            message(STATUS "FMA requires enabling whole program optimization. FMA check disabled.")
            set(ALLOW_FMA FALSE)
        endif()
    endif()

    if(SIMD_FMA AND ALLOW_FMA)
        # Set substitution variables for configuration file
        set(FRYDOM_HAS_FMA "#define FRYDOM_HAS_FMA")
    endif()

    # Figure out NEON support

    # Set substitution variables for configuration file.
    if(SIMD_NEON)
        set (FRYDOM_HAS_NEON "#define FRYDOM_HAS_NEON")
    endif()

    # Add SIMD flags to Chrono compiler flags
    # Note that these flags are already added to CMake compiler flags
    set(FR_C_FLAGS "${FR_C_FLAGS} ${SIMD_C_FLAGS}")
    set(FR_CXX_FLAGS "${FR_CXX_FLAGS} ${SIMD_CXX_FLAGS}")

else()

    message(STATUS "SIMD support disabled")

endif()

#-----------------------------------------------------------------------------
# Set the base compilation flags
#-----------------------------------------------------------------------------

message(STATUS "Compiler and linker flags:")
message(STATUS "  C++ compiler flags: ${FR_CXX_FLAGS}")
message(STATUS "  C compiler flags:   ${FR_C_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FR_CXX_FLAGS}")

# =============================================================================
# FRyDoM DEPENDENCIES
# =============================================================================

include(Add_Eigen3)
message(STATUS "eigen3 : " ${eigen3_SOURCE_DIR})
set(EIGEN3_INCLUDE_DIR ${eigen3_SOURCE_DIR})
include(Add_mathutils)
include(Add_Boost)
include(Add_chrono)

##CC debug
#message(STATUS "debug : CMAKE_CXX_FLAGS : "${CMAKE_CXX_FLAGS})
#message(STATUS "debug : CH_CXX_FLAGS = "${CH_CXX_FLAGS})
#exit()
##CC

include(Add_hdb5_io)
include(Add_GeographicLib)
include(Add_cppfs)
include(Add_TimeZone)
include(Add_hermes)
include(Add_json)
include(Add_spdlog)
include(Add_acme)

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
