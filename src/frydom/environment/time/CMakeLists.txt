message(STATUS "        ...Environment/time")

target_sources(frydom PRIVATE
        FrTimeZone.cpp
        FrTimeServices.cpp
        )

