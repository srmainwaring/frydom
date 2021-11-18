message(STATUS "    ...radiation")

target_sources(frydom PRIVATE
        FrRadiationForce.cpp
        FrRadiationModel.cpp
        FrVariablesBEMBodyBase.cpp
        FrRadiationModelBase.cpp
        FrRadiationConvolutionModel.cpp
        FrRadiationRecursiveConvolutionModel.cpp
        FrRadiationConvolutionModelForwardSpeedCorrection.cpp
        )
