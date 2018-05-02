

#find_package(yaml-cpp QUIET)
if (NOT yaml-cpp_FOUND)
    include(FetchContent)

    set(yaml-cpp_URL "https://github.com/jbeder/yaml-cpp.git")
    FetchContent_Declare(yaml-cpp
            GIT_REPOSITORY ${yaml-cpp_URL}
            GIT_TAG "yaml-cpp-0.6.2"
            )

    FetchContent_GetProperties(yaml-cpp)
    message(STATUS yaml-cpp)
    message(STATUS ${yaml-cpp_POPULATED})
    message(STATUS ${yaml-cpp_SOURCE_DIR})
    message(STATUS ${yaml-cpp_BINARY_DIR})
    if(NOT yaml-cpp_POPULATED)
        message(STATUS "Downloading, Configuring and Generating 'yaml-cpp' dependency")
        FetchContent_Populate(yaml-cpp)

        # yaml-cpp BUILD OPTIONS
        set(BUILD_SHARED_LIBS TRUE)
        set(YAML_CPP_BUILD_TESTS OFF)
        set(YAML_CPP_BUILD_TOOLS OFF)
        set(YAML_CPP_BUILD_CONTRIB OFF)


        # TODO: ici, on peut reconstuire la target yaml-cpp a base de find_path et find_library

        add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})

        message(STATUS "DONE")
    endif()
endif()


if (TARGET yaml-cpp)
#    message(STATUS "FOUND yaml-cpp TARGET")
    get_target_property(INC yaml-cpp INCLUDE_DIRECTORIES)
    message(STATUS "Found yaml-cpp : ${INC}")
#    target_include_directories(yaml-cpp INTERFACE ${INC})
#
#    include_directories(${INC})  # FIXME: pourquoi n'a-t-on pas l'include dans la target ???

endif()