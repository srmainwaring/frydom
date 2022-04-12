
# Here we declare the different PATH, TAG and PATCH to get the FRyDoM dependencies

# Chrono
set(chrono_URL "https://github.com/projectchrono/chrono/archive/refs/tags/4.0.0.tar.gz")
set(chrono_PATCH chrono_V4.0.0.patch)

# GeographicLib
set(geographiclib_URL https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.52.tar.gz/download)
set(geographiclib_TAG v1.52 CACHE STRING "geographiclib")
set(geographiclib_PATCH GeographicLib_1_52.patch)

# GoogleTest
set(googletest_URL https://github.com/google/googletest.git)
set(googletest_TAG release-1.10.0 CACHE STRING "googletest version")

# hermes
set(hermes_URL git@frydom-ce.org:ce/hermes.git)
set(hermes_TAG v1.4 CACHE STRING " version")

# HDB5_IO
set(hdb5_io_URL git@frydom-ce.org:ce/hdb5-io.git)
set(hdb5_io_TAG v2.6.1 CACHE STRING "hdb5_io version")

# TimeZone
set(timezone_URL https://github.com/HowardHinnant/date.git)
#set(timezone_TAG v2.4.1)
set(timezone_TAG 9454aeda2bd6b23c5dabe997a6b392cef88e321d CACHE STRING "timezone version")

# CPPFS
set(cppfs_URL https://github.com/cginternals/cppfs.git)
set(cppfs_TAG v1.2.0 CACHE STRING "cppfs version")

# JSON
set(json_URL https://github.com/nlohmann/json.git)
set(json_TAG v3.6.1 CACHE STRING "json version")

# spdlog
set(spdlog_URL https://github.com/gabime/spdlog.git)
set(spdlog_TAG v1.7.0 CACHE STRING "spdlog version")

# cxxopts
set(cxxopts_URL https://github.com/jarro2783/cxxopts.git)
set(cxxopts_TAG v2.2.0 CACHE STRING "cxxopts version")

## HDF5
#set(HDF5_URL https://github.com/HDFGroup/hdf5.git)
#set(HDF5_TAG hdf5-1_10_6 CACHE STRING "HDF5 version")

# acme
set(acme_URL git@frydom-ce.org:ce/acme.git)
set(acme_TAG feature/LL_refacto_prop_rudder CACHE STRING "acme version")

### Indirect dependencies / already included

# MathUtils
set(mathutils_URL git@frydom-ce.org:ce/mathutils.git)
set(mathutils_TAG v1.10 CACHE STRING "mathutils version")
set(MATHUTILS_BUILD_TESTS OFF CACHE BOOL "")
set(MATHUTILS_BUILD_BOOST_TESTS OFF CACHE BOOL "")

# OpenMesh
set(openmesh_URL https://www.graphics.rwth-aachen.de:9000/OpenMesh/OpenMesh.git)
set(openmesh_TAG OpenMesh-8.0 CACHE STRING "openmesh version")

#Meshoui
set(meshoui_TAG v1.3.2 CACHE STRING "meshoui version")
set(MESHOUI_BUILD_TESTS OFF CACHE BOOL "")
set(MESHOUI_BUILD_TOOLS OFF CACHE BOOL "")

# Boost
set(boost_URL https://boostorg.jfrog.io/artifactory/main/release/1.75.0/source/boost_1_75_0.tar.gz)
set(boost_TAG 1.75)
