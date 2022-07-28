message(STATUS "    ...buoyancy")

target_sources(frydom PRIVATE
        FrBarElement.cpp
        FrBuoyancyBarElements.cpp
        BuoyancyForce.cpp
        )