
message(STATUS "    ...DEV TESTS :")

set(REFACTORING_TESTS
        test_FrLinks
        test_cylinder_interaction
        test_FrFunction
        test_FrFileSystem
        test_spdlog
        test_MorisonExtended
        test_MorisonExtFreeMotion
        test_lumped_mass_cable
        test_chrono_spring_bodies
        test_LM_element
        test_catenary_clump_weight
        test_catenary_seabed
        test_pendulum_lumped_mass
        test_bspline
        test_beam_IGA
        test_fea_cable
        test_fea_cable_discretization
        test_fea_mooring_line_with_clump
        test_vertical_fea_cable_in_waves
        )

foreach (test ${REFACTORING_TESTS})
    message(STATUS "        ...add ${test}")

    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} FRyDoM_Engine)

endforeach ()

set(subdirs
        # add here the test source subdirectories
        )

foreach (subdir ${subdirs})
    add_subdirectory(${subdir})
endforeach ()

