#find_package(HDF5 REQUIRED COMPONENTS CXX)
#
#message(STATUS ${HDF5_INCLUDE_DIRS})
#message(STATUS ${HDF5_LIBRARIES})
#
#add_library(HDF5 INTERFACE)
#target_include_directories(HDF5 INTERFACE ${HDF5_INCLUDE_DIRS})
#target_link_libraries(HDF5 INTERFACE ${HDF5_LIBRARIES})


include(FetchContent)

FetchContent_Declare(HDF5
        GIT_REPOSITORY ${HDF5_URL}
        GIT_TAG ${HDF5_TAG}
        )

FetchContent_GetProperties(HDF5)

if (NOT HDF5_POPULATED)
    message(STATUS "Downloading, Configuring and Generating 'HDF5' dependency")
    FetchContent_Populate(HDF5)

    # HDF5 BUILD OPTIONS
    set(HDF5_BUILD_CPP_LIB ON CACHE BOOL "" FORCE)
    set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE) # TODO: essayer et voir comment ca impacte...
#    set(HDF5_ENABLE_Z_LIB_SUPPORT OFF CACHE BOOL "" FORCE)
#    set(HDF5_ENABLE_SZIP_SUPPORT OFF CACHE BOOL "" FORCE)
#    set(HDF5_ENABLE_SZIP_ENCODING OFF CACHE BOOL "" FORCE)


    add_subdirectory(${hdf5_SOURCE_DIR} ${hdf5_BINARY_DIR})
else ()
    message(STATUS "HDF5 already populated")
endif ()

#message(STATUS: HDF5 target ${HDF5_LIB_TARGET})


if (TARGET hdf5-static)
    message(STATUS "HDF5 TARGET FOUND")
    get_target_property(INC hdf5-static INCLUDE_DIRECTORIES)
#    target_include_directories(hdf5-static PUBLIC ${INC})
    message(${INC})
else()
    message(STATUS "hdf5-static target NOT FOUND")
endif()

if (TARGET hdf5_cpp-static)
    message(STATUS "HDF5_CPP TARGET FOUND")
    get_target_property(INC hdf5_cpp-static INCLUDE_DIRECTORIES)
#    target_include_directories(HDF5 PUBLIC ${INC})
    message(${INC})
else()
    message(STATUS "hdf5-static target NOT FOUND")
endif()

include_directories(${hdf5_BINARY_DIR}) # See why this directory is not included into the target...
