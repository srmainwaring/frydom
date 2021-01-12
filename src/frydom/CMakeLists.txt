
message(STATUS "===================================================================")
message(STATUS "Getting source files for FRyDoM...")
message(STATUS "===================================================================")


set(FRYDOM_SRC_FILES "")

# Adding a new subdirectory to the FRyDoM root source directory must be done by adding its name here
set(subdirs
        logging
        asset
        cable
        collision
        core
        environment
        hydrodynamics
        io
        mesh
        utils
        )

# TODO: reposer plutot sur target_sources()
# voir crascit.com/2016/01/31/enhanced-source-file-handling-with-target_sources/

set(FRYDOM_SRC_FILES "" cable/common/FrCableShapeInitializer.cpp cable/common/FrCableShapeInitializer.h cable/mooring_components/FrClumpWeight.cpp cable/mooring_components/FrClumpWeight.h cable/catenary/FrCatenaryLineSeabed.cpp cable/catenary/FrCatenaryLineSeabed.h core/contact/FrContactInc.h core/contact/FrContactSMC.cpp core/contact/FrContactSMC.h core/contact/FrContactNSC.cpp core/contact/FrContactNSC.h)
foreach (subdir ${subdirs})

    add_subdirectory(${subdir})
    string(TOUPPER FRYDOM_${subdir}_SOURCES SOURCE_FILES)
    list(APPEND FRYDOM_SRC_FILES ${${SOURCE_FILES}})

endforeach ()

list(APPEND FRYDOM_SRC_FILES "${CMAKE_CURRENT_BINARY_DIR}/version.cpp")

# =============================================================================
# FRyDoM DEPENDENCIES
# =============================================================================

# FRyDoM main shared library
add_library(FRyDoM_Engine SHARED ${FRYDOM_SRC_FILES})
target_link_libraries(FRyDoM_Engine
        ChronoEngine
        MathUtils::MathUtils
        hdb5io
        GeographicLib
        fmt
        cppfs
        timezone
        hermes
        OpenMeshCore
        nlohmann_json
        spdlog::spdlog_header_only
        )
if(frydom_use_irrlicht)
    target_link_libraries(FRyDoM_Engine ChronoEngine_irrlicht)
else()
    target_compile_definitions(FRyDoM_Engine PUBLIC -DH5_NO_IRRLICHT)
endif()

if (frydom_build_on_LIGER)
    target_compile_definitions(FRyDoM_Engine PUBLIC -DH5_NO_UUID)
endif()

target_include_directories(FRyDoM_Engine PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        )

target_compile_definitions(FRyDoM_Engine PUBLIC
        RESOURCES_VIZU_PATH="${PROJECT_SOURCE_DIR}/data/"
        )

set_target_properties( FRyDoM_Engine
        PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/frydom"
        )

message(STATUS " RESOURCES_VIZU_PATH=${PROJECT_SOURCE_DIR}/data/")

get_target_property(INC FRyDoM_Engine INTERFACE_LINK_LIBRARIES)
message(STATUS "FRyDoM_INCLUDE_DIRECTORIES : ${INC}")


# TODO: voir si on passe ce CMakeLists.txt dans le repertoire parent pour ne pas avoir a faire .. dans l'include
