message(STATUS "    ...propulsion")

target_sources(frydom PRIVATE
        FrPropellerForce.cpp
        FrFirstQuadrantPropellerForce.cpp
        FrCPPForce.cpp
        FrRudderForce.cpp
        FrFlapRudderForce.cpp
        )
