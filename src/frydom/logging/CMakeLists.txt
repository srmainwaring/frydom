message(STATUS "    ...logging")

target_sources(frydom PRIVATE
        FrLogManager.cpp
        FrPathManager.cpp
        FrEventLogger.cpp
        FrLoggable.cpp
        FrTypeNames.cpp
        FrPathPolicies.cpp
        FrCastor.cpp
        )
