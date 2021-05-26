message(STATUS "    ...damping")

target_sources(frydom PRIVATE
        FrLinearDamping.cpp
        FrQuadraticDamping.cpp
        )
