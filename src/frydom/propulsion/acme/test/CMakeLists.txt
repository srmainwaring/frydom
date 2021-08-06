add_executable(test_FPP1Q test_FPP1Q.cpp)

target_link_libraries(test_FPP1Q acme)

set_target_properties(test_FPP1Q PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)