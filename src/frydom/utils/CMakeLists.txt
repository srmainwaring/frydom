message(STATUS "    ...utils")

target_sources(frydom PRIVATE
        FrFileSystem.cpp
        FrConfig.cpp
        )

if(frydom_use_irrlicht)

    target_sources(frydom PRIVATE
            FrIrrApp.cpp
            FrIrrCamera.cpp
            )
endif()
