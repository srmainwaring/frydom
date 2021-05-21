message(STATUS "    ...hydrostatic")

target_sources(frydom PRIVATE
        FrLinearHydrostaticForce.cpp
        FrLinearHydrostaticStiffnessMatrix.cpp
        FrNonlinearHydrostaticForce.cpp
        FrHydrostaticEquilibrium.cpp
        FrHydrostaticsProperties.cpp
        )

