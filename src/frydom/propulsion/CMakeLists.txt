message(STATUS "    ...propulsion")

target_sources(frydom PRIVATE
        FrPropellerForce.cpp
        FrCPPForce.cpp
        FrRudderForce.cpp
        FrFlapRudderForce.cpp
        )
