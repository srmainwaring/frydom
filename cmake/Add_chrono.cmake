include(FetchContent)

FetchContent_Declare(chrono
        URL ${chrono_URL}
        PATCH_COMMAND patch -p1 < ${PROJECT_SOURCE_DIR}/cmake/patches/${chrono_PATCH}
        )


FetchContent_GetProperties(chrono)
if (NOT chrono_POPULATED)

    message(STATUS "******* FETCHING chrono dependency from ${PROJECT_NAME} (requested version: ${chrono_TAG}) *******")
    FetchContent_Populate(chrono)

#    list(APPEND CMAKE_MODULE_PATH "${chrono_SOURCE_DIR}")

    # chrono BUILD OPTIONS
    set(CMAKE_SOURCE_DIR ${chrono_SOURCE_DIR})
    set(BUILD_TESTS FALSE CACHE BOOL "" FORCE)
    set(BUILD_DEMOS FALSE CACHE BOOL "" FORCE)
    set(BUILD_DEMOS_BASE FALSE CACHE BOOL "" FORCE)
    set(BUILD_DEMOS_FEA FALSE CACHE BOOL "" FORCE)
    set(BUILD_DEMOS_IRRLICHT FALSE CACHE BOOL "" FORCE)
    set(BUILD_DEMOS_POSTPROCESS FALSE CACHE BOOL "" FORCE)
    set(BUILD_TESTING FALSE CACHE BOOL "" FORCE)
    set(BUILD_TESTS_BASE FALSE CACHE BOOL "" FORCE)
    set(BUILD_TESTS_FEA FALSE CACHE BOOL "" FORCE)
    set(BUILD_BENCHMARKING FALSE CACHE BOOL "" FORCE)

    set(ENABLE_MODULE_CASCADE OFF CACHE BOOL "" FORCE)
    set(ENABLE_MODULE_COSIMULATION OFF CACHE BOOL "" FORCE)
    set(ENABLE_MODULE_FEA ON CACHE BOOL "" FORCE)
    set(ENABLE_MODULE_FSI OFF CACHE BOOL "" FORCE)
    set(ENABLE_MODULE_IRRLICHT ON CACHE BOOL "" FORCE)
    set(ENABLE_MODULE_MATLAB OFF CACHE BOOL "" FORCE)
    set(ENABLE_MODULE_MKL OFF CACHE BOOL "" FORCE)
    set(ENABLE_MODULE_OPENGL OFF CACHE BOOL "" FORCE)
    set(ENABLE_MODULE_PARALLEL OFF CACHE BOOL "" FORCE)
    set(ENABLE_MODULE_POSTPROCESS ON CACHE BOOL "" FORCE)
    set(ENABLE_MODULE_PYTHON OFF CACHE BOOL "" FORCE)
    set(ENABLE_MODULE_VEHICLE OFF CACHE BOOL "" FORCE)
    set(ENABLE_OPENMP ON CACHE BOOL "" FORCE)

    if (FRYDOM_USE_IRRLICHT)
        set(ENABLE_MODULE_IRRLICHT ON CACHE BOOL "" FORCE)
    else ()
        set(ENABLE_MODULE_IRRLICHT OFF CACHE BOOL "" FORCE)
    endif ()


    add_subdirectory(${chrono_SOURCE_DIR} ${chrono_BINARY_DIR})

    set(chrono_DIR ${chrono_BINARY_DIR}/cmake)

endif ()

if (TARGET ChronoEngine)
    message(STATUS "ChronoEngine TARGET FOUND")
    get_target_property(INC ChronoEngine INCLUDE_DIRECTORIES)
    # Chrono cmake does not embed include directories into its targets... Adding them
    target_include_directories(ChronoEngine PUBLIC ${INC})
endif ()

if (TARGET ChronoEngine_fea)
    message(STATUS "ChronoEngine_fea TARGET FOUND")
    get_target_property(INC ChronoEngine_fea INCLUDE_DIRECTORIES)
    # Chrono cmake does not embed include directories into its targets... Adding them
    target_include_directories(ChronoEngine_fea PUBLIC ${INC})
endif ()

if (TARGET ChronoEngine_irrlicht)
    message(STATUS "ChronoEngine_irrlicht TARGET FOUND")
    get_target_property(INC ChronoEngine_irrlicht INCLUDE_DIRECTORIES)
    # Chrono cmake does not embed include directories into its targets... Adding them
    target_include_directories(ChronoEngine_irrlicht PUBLIC ${INC})
endif ()

if (TARGET ChronoEngine_postprocess)
    message(STATUS "ChronoEngine_postprocess TARGET FOUND")
    get_target_property(INC ChronoEngine_postprocess INCLUDE_DIRECTORIES)
    # Chrono cmake does not embed include directories into its targets... Adding them
    target_include_directories(ChronoEngine_postprocess PUBLIC ${INC})
endif ()

# TODO: ajouter une custom_target rassemblat toutes les libs delectionnees pour Chrono avec des add_dependencies()
