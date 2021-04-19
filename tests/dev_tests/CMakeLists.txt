
message(STATUS "    ...DEV TESTS :")

set(REFACTORING_TESTS
        test_FrLinks
        test_cylinder_interaction
        test_FrFunction
        test_FrFileSystem
        test_spdlog
        test_MorisonExtended
        test_MorisonExtFreeMotion
#        test_lumped_mass_cable
        test_chrono_spring_bodies
        test_catenary_clump_weight
        test_catenary_seabed
        test_pendulum_lumped_mass
        test_bspline
        test_beam_IGA
        test_fea_cable
        test_fea_cable_discretization
        test_fea_mooring_line_with_clump
        test_vertical_fea_cable_in_waves
        bench_FEACable_moored
        bench_FEACable_FOWT_moored
        test_log_hdf5
        )

foreach (test ${REFACTORING_TESTS})
    message(STATUS "        ...add ${test}")

    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} frydom)

    set_target_properties( ${test}
            PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
            )

endforeach ()
