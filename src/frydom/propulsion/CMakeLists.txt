message(STATUS "    ...propulsion")

add_subdirectory(acme)

target_sources(frydom PRIVATE
        FrActuatorForceBase.cpp
        FrPropellerForce.cpp
        FrFirstQuadrantPropellerForce.cpp
        FrFourQuadrantPropellerForce.cpp
        FrCPPForce.cpp
        FrRudderForce.cpp
        FrFlapRudderForce.cpp
        FrPropellerRudder.cpp
        )
