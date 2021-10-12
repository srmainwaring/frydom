message(STATUS "    ...propulsion")

target_sources(frydom PRIVATE
        FrActuatorForceBase.cpp
        FrPropeller.cpp
        FrRudder.cpp
        FrPropellerRudder.cpp
        )
