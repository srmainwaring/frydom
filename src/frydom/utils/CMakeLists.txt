message(STATUS "    ...utils")

target_sources(frydom PRIVATE
        FrFileSystem.cpp
        FrConfig.cpp
        )

if(FRYDOM_USE_IRRLICHT)

    target_sources(frydom PRIVATE
            FrIrrApp.cpp
            FrIrrCamera.cpp
            )
endif()
