
message(STATUS "    ...VALIDATION CASES :")

set(VALIDATION_CASES
        bench_BoxDecayTests
        bench_DTMB5512
        bench_HS_equilibrium
        bench_SphereDecay
        bench_SphereIrregular
        bench_SphereRegular
        bench_StabilityAnalysis
        )

foreach (bench ${VALIDATION_CASES})
    message(STATUS "        ...add ${bench}")

    add_executable(${bench} ${bench}.cpp)
    target_link_libraries(${bench} FRyDoM_Engine)

endforeach ()


add_subdirectory(cables)
