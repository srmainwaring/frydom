add_library(acme)
target_link_libraries(acme
        MathUtils::MathUtils
        nlohmann_json)
target_include_directories(acme PUBLIC ${CMAKE_CURRENT_LIST_DIR}/..)

add_subdirectory(thruster)
add_subdirectory(rudder)
add_subdirectory(sail)

add_subdirectory(test)
