message(STATUS "    ...tests")

message(STATUS "Test programs for FRyDoM framework...")


# Resource data loading / path definition
set(FRYDOM_DATA_FOLDER ${CMAKE_SOURCE_DIR}/data)


set(FRYDOM_LOG_FOLDER ${CMAKE_CURRENT_BINARY_DIR}/../logs)
configure_file(frydom_config.in ${CMAKE_CURRENT_BINARY_DIR}/../bin/.frydom_config)

# FIXME : reactivate options?
if (${FRYDOM_BUILD_DEV_TESTS})
#    set(FRYDOM_LOG_FOLDER ${CMAKE_CURRENT_BINARY_DIR}/dev_tests)
#    configure_file(frydom_config.in ${CMAKE_CURRENT_BINARY_DIR}/dev_tests/.frydom_config)
    add_subdirectory(dev_tests)
endif ()

if (${FRYDOM_BUILD_BENCHES})
#    set(FRYDOM_LOG_FOLDER ${CMAKE_CURRENT_BINARY_DIR}/validations)
    if(${CMAKE_BUILD_TYPE} MATCHES Release)
        configure_file(frydom_config.in ${CMAKE_CURRENT_SOURCE_DIR}/validations/.frydom_config)
    endif ()
    add_subdirectory(validations)
endif ()

if (${FRYDOM_BUILD_DEMOS})
#    set(FRYDOM_LOG_FOLDER ${CMAKE_CURRENT_BINARY_DIR}/demo)
#    configure_file(frydom_config.in ${CMAKE_CURRENT_BINARY_DIR}/demos/.frydom_config)
    add_subdirectory(demos)
endif ()

if (${FRYDOM_BUILD_UNIT_TESTS})
    set(FRYDOM_LOG_FOLDER ${CMAKE_CURRENT_BINARY_DIR}/unit_tests)
    configure_file(frydom_config.in ${CMAKE_CURRENT_BINARY_DIR}/unit_tests/.frydom_config)
    add_subdirectory(unit_tests)
endif ()

