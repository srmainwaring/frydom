
message(STATUS "    ...DEMOS :")

# Name executable

set(DEMOS
        Ballistic
        Environment
        FreeSurface
        Body
        Rotation_and_Frame
        Hydrodynamic
        Morison
        Cable
        Hub_Installation
        Langlee
        FOSWEC
        MooringLines
        NonLinearHydrostatics
        Constraints
        Links
        Tensegrity
        HexagonalArticulatedBuoy
        )

# Compile target
foreach (test ${DEMOS})
    message(STATUS "        ...add demo_${test}")
    add_executable(demo_${test} demo_${test}.cpp)
    target_link_libraries(demo_${test} frydom)

    set_target_properties( demo_${test}
            PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
            )
endforeach ()

#foreach (test ${BENCH})
#    message(STATUS "        ...add bench_${test}")
#    add_executable(bench_${test} bench_${test}.cpp)
#    target_link_libraries(bench_${test} FRyDoM_Engine)
#endforeach ()

#set(subdirs
#        # add here the test source subdirectories
#        )
#
#foreach (subdir ${subdirs})
#    add_subdirectory(${subdir})
#endforeach ()

