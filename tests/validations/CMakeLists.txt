
message(STATUS "    ...VALIDATION CASES :")

set(VALIDATION_CASES
        BoxDecayTests
        DTMB5512
        HS_equilibrium
        SphereDecay
        SphereIrregular
        SphereRegular
        StabilityAnalysis
        Recursive_convolution
        MorisonCrossingElement
        MorisonSingleImmersedElement
        FixedJacketStructure
        SpreadMooredFPSO
        PYW_Comparison_direct_recursive_convolutions
        )

foreach (test ${VALIDATION_CASES})
    message(STATUS "        ...add bench_${test}")

    add_executable(bench_${test} bench_${test}.cpp)
    target_link_libraries(bench_${test} frydom)

    set_target_properties( bench_${test}
            PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
            )

endforeach ()


add_subdirectory(cables)
