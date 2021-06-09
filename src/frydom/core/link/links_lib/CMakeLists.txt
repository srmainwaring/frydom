add_subdirectory(actuators)

target_sources(frydom PRIVATE
        FrCylindricalLink.cpp
        FrDOFMaskLink.cpp
        FrFixedLink.cpp
        FrFreeLink.cpp
        FrLink.cpp
        FrPrismaticLink.cpp
        FrPrismaticRevoluteLink.cpp
        FrRevoluteLink.cpp
        FrScrewLink.cpp
        FrSphericalLink.cpp
        )