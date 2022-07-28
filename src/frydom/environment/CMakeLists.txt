message(STATUS "    ...Environment")

add_subdirectory(time)
add_subdirectory(ocean)
add_subdirectory(atmosphere)
add_subdirectory(flow)
add_subdirectory(geographicServices)

target_sources(frydom PRIVATE FrEnvironment.cpp)
