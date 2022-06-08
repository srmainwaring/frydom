message(STATUS "        ...Atmosphere")

add_subdirectory(wind)

target_sources(frydom PRIVATE FrAtmosphere.cpp)
