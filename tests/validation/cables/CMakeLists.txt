
message(STATUS "       ...CABLE VALIDATION CASES:")

set(CABLE_VALIDATION_CASES
        bench_FEACable_Orcaflex
        bench_Cables
        )

foreach (bench ${CABLE_VALIDATION_CASES})
    message(STATUS "        ...add ${bench}")

    add_executable(${bench} ${bench}.cpp)
    target_link_libraries(${bench} FRyDoM_Engine)

endforeach ()

