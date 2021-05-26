message(STATUS "    ...excitation")

target_sources(frydom PRIVATE
        FrLinearExcitationForce.cpp
        FrNonLinearFroudeKrylovForce.cpp
        FrLinearHDBForce.cpp
        FrLinearDiffractionForce.cpp
        FrLinearFroudeKrylovForce.cpp
        )

