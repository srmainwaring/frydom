message(STATUS "                ...waves")

add_subdirectory(airy)

target_sources(frydom PRIVATE
        FrWaveDispersionRelation.cpp
        FrWaveField.cpp
        FrWaveSpectrum.cpp
        FrKinematicStretching.cpp
        )
