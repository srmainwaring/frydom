message(STATUS "    ...manoeuvring")

target_sources(frydom PRIVATE
        FrManoeuvringDamping.cpp
        FrSutuloManoeuvringForce.cpp
        FrHullResistance.cpp
        FrAbkowitzManoeuvringForce.cpp
        )
