
message(STATUS "       ...CABLE VALIDATION CASES:")

set(CABLE_VALIDATION_CASES
        FEACable_Orcaflex
        Cables
        )

foreach (test ${CABLE_VALIDATION_CASES})
    message(STATUS "        ...add bench_${test}")

    add_executable(bench_${test} bench_${test}.cpp)
    target_link_libraries(bench_${test} FRyDoM_Engine)

    set_target_properties( bench_${test}
            PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
            )

endforeach ()

