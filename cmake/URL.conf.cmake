
# Here we declare the different PATH, TAG and PATCH to get the FRyDoM dependencies

# Chrono
set(chrono_URL "https://github.com/projectchrono/chrono.git")
set(chrono_TAG 4.0.0 CACHE STRING "Chrono version")
set(chrono_PATCH chrono_V4.0.0.patch)


# GeographicLib
set(geographiclib_URL https://github.com/D-ICE/geographiclib.git)
set(geographiclib_TAG v1.49 CACHE STRING "geographiclib")
set(geographiclib_PATCH GeographicLib.patch)


# GoogleTest
set(googletest_URL https://github.com/google/googletest.git)
set(googletest_TAG release-1.8.1 CACHE STRING "googletest version")


# hermes
set(hermes_URL git@frydom-ce.org:ce/hermes.git)
set(hermes_TAG v1.3.1 CACHE STRING " version")


# HDB5_IO
set(hdb5io_URL git@frydom-ce.org:ce/hdb5-io.git)
set(hdb5io_TAG v1.0.2 CACHE STRING "hdb5io version")


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


# HDF5
set(HDF5_URL https://github.com/HDFGroup/hdf5.git)
set(HDF5_TAG hdf5-1_8_21 CACHE STRING "HDF5 version")


### Indirect dependencies / already included

# MathUtils
set(mathutils_URL git@frydom-ce.org:ce/mathutils.git)
set(mathutils_TAG v1.5 CACHE STRING "mathutils version")
set(MATHUTILS_BUILD_TESTS OFF CACHE BOOL "")
set(MATHUTILS_BUILD_BOOST_TESTS OFF CACHE BOOL "")


# OpenMesh
set(openmesh_URL https://www.graphics.rwth-aachen.de:9000/OpenMesh/OpenMesh.git)
set(openmesh_TAG OpenMesh-8.0 CACHE STRING "openmesh version")


## MeshOui.
#set(meshoui_URL git@frydom-ce.org:ce/meshoui.git)
#set(meshoui_TAG v1.0.2 CACHE STRING "meshoui version")
#set(MESHOUI_BUILD_TESTS OFF CACHE BOOL "")