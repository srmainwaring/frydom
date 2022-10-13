
message(STATUS "===================================================================")
message(STATUS "Getting source files for FRyDoM...")
message(STATUS "===================================================================")

add_library(frydom)


add_subdirectory(logging)
add_subdirectory(asset)
add_subdirectory(cable)
add_subdirectory(collision)
add_subdirectory(core)
add_subdirectory(environment)
add_subdirectory(hydrodynamics)
add_subdirectory(io)
add_subdirectory(mesh)
add_subdirectory(propulsion)
add_subdirectory(utils)

target_sources(frydom PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/version.cpp")


# =============================================================================
# FRyDoM DEPENDENCIES
# =============================================================================

add_dependencies(frydom check_git_${PROJECT_NAME}) # For git_watcher to fetch git informations before effective build

target_link_libraries(frydom PUBLIC
        Boost::boost
        MathUtils::MathUtils
        hdb5_io
        GeographicLib
        fmt
        cppfs
        date-tz
        hermes
        OpenMeshCore
        nlohmann_json
        spdlog::spdlog_header_only
        acme
        )

if (build_boost)
    add_dependencies(frydom boost)
endif ()

target_link_libraries(frydom PUBLIC ChronoEngine)

#target_link_libraries(frydom PUBLIC ChronoEngine_pardisomkl)

target_compile_options(frydom PUBLIC -Wno-deprecated-declarations)
target_compile_options(frydom PUBLIC -Wimplicit-fallthrough)
#target_compile_options(frydom PRIVATE -Wextra)

if (NOT BUILD_SHARED_LIBS)
    set_property(TARGET frydom PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()

if (FRYDOM_USE_IRRLICHT)
    target_link_libraries(frydom PUBLIC ChronoEngine_irrlicht)
else ()
    target_compile_definitions(frydom PUBLIC -DH5_NO_IRRLICHT)
endif ()

if (FRYDOM_BUILD_ON_LIGER)
    target_compile_definitions(frydom PUBLIC -DH5_NO_UUID)
endif ()

target_include_directories(frydom PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        )

target_compile_definitions(frydom PUBLIC
        RESOURCES_VIZU_PATH="${PROJECT_SOURCE_DIR}/data/"
        )

set_target_properties(frydom
        PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/frydom"
        )

message(STATUS " RESOURCES_VIZU_PATH=${PROJECT_SOURCE_DIR}/data/")

get_target_property(INC frydom INTERFACE_LINK_LIBRARIES)
message(STATUS "FRyDoM_INCLUDE_DIRECTORIES : ${INC}")


# TODO: voir si on passe ce CMakeLists.txt dans le repertoire parent pour ne pas avoir a faire .. dans l'include
