message(STATUS "                    ...airy")

target_sources(frydom PRIVATE
        FrAiryRegularWaveField.cpp
        FrAiryRegularOptimWaveField.cpp
        FrAiryIrregularWaveField.cpp
        FrAiryIrregularOptimWaveField.cpp
        )
