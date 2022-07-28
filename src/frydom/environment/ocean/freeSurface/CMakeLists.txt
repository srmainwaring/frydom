message(STATUS "            ...FreeSurface")

add_subdirectory(tidal)
add_subdirectory(waves)

target_sources(frydom PRIVATE
        FrFreeSurface.cpp
        )
