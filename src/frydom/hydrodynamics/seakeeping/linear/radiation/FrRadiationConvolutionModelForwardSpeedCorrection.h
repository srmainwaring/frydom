// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================

#ifndef FRYDOM_FRRADIATIONCONVOLUTIONMODELFORWARDSPEEDCORRECTION_H
#define FRYDOM_FRRADIATIONCONVOLUTIONMODELFORWARDSPEEDCORRECTION_H

#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrRadiationConvolutionModel.h"

namespace frydom {

  /// This class computes the forward speed correction of the convolution term by direct integration of the convolution of the IRF and the body velocity.
  class FrRadiationConvolutionModelForwardSpeedCorrection : public FrRadiationConvolutionModel {

   public:
    virtual ~FrRadiationConvolutionModelForwardSpeedCorrection();

    /// Constructor of the class.
    FrRadiationConvolutionModelForwardSpeedCorrection(const std::string &name, FrOffshoreSystem *system, std::shared_ptr<FrHydroDB> HDB);

   private:

    /// This method computes the radiation loads with the forward speed correction.
    void Compute(double time) override;

    /// This method computes the the forward speed dependent part of the convolution term.
    GeneralizedForce ForwardSpeedCorrection(FrBEMBody *BEMBody) const;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  std::shared_ptr<FrRadiationConvolutionModel>
  make_radiation_convolution_model_with_forward_speed_correction(const std::string &name, FrOffshoreSystem *system, std::shared_ptr<FrHydroDB> HDB);


}


#endif //FRYDOM_FRRADIATIONCONVOLUTIONMODELFORWARDSPEEDCORRECTION_H
