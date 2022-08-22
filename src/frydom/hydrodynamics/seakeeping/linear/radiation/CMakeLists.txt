message(STATUS "    ...radiation")

target_sources(frydom PRIVATE
        FrRadiationForce.cpp
        FrRadiationModel.cpp
        FrVariablesBEMBodyBase.cpp
        FrRadiationModelBaseVariable.cpp
        FrRadiationModelBaseKRM.cpp
        FrRadiationConvolutionModel.cpp
        FrRadiationRecursiveConvolutionModel.cpp
        FrRadiationConvolutionModelForwardSpeedCorrection.cpp
        FrRadiationRecursiveConvolutionModelForwardSpeedCorrection.cpp
        FrAddedMass.cpp
        )
