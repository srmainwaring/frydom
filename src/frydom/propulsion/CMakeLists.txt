message(STATUS "    ...propulsion")

target_sources(frydom PRIVATE
        FrPropulsionForce.cpp
        FrRudderForce.cpp
        FrFlapRudderForce.cpp
        )
