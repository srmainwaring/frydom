message(STATUS "    ...hydrodynamics")

add_subdirectory(damping)
add_subdirectory(hydrostatic)
add_subdirectory(manoeuvring)
add_subdirectory(morison)
add_subdirectory(seakeeping)
add_subdirectory(wave_resistance)
add_subdirectory(buoyancy)

target_sources(frydom PRIVATE
        FrEquilibriumFrame.cpp
        )

