message(STATUS "    ...propulsion")

target_sources(frydom PRIVATE
        FrPropulsionActuator.cpp
        FrPropellerForce.cpp
        FrFirstQuadrantPropellerForce.cpp
        FrFourQuadrantPropellerForce.cpp
        FrCPPForce.cpp
        FrRudderForce.cpp
        FrFlapRudderForce.cpp
        FrPropellerRudder.cpp
        )
