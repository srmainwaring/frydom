message(STATUS "    ...propulsion")

target_sources(frydom PRIVATE
        FrPropellerForce.cpp
        FrFirstQuadrantPropellerForce.cpp
        FrFourQuadrantPropellerForce.cpp
        FrCPPForce.cpp
        FrRudderForce.cpp
        FrFlapRudderForce.cpp
        FrPropellerRudder.cpp
        )
