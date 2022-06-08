message(STATUS "        ...Ocean")

add_subdirectory(current)
add_subdirectory(seabed)
add_subdirectory(freeSurface)

target_sources(frydom PRIVATE
        FrOcean.cpp
        )
