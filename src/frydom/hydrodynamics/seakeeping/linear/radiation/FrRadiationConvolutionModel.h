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

#ifndef FRYDOM_FRRADIATIONCONVOLUTIONMODEL_H
#define FRYDOM_FRRADIATIONCONVOLUTIONMODEL_H

#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrRadiationModel.h"

namespace frydom {

  /// This class computes the convolution term by direct integration of the convolution of the IRF and the body velocity.
  class FrRadiationConvolutionModel : public FrRadiationModel{

   private:
    std::unordered_map<FrBEMBody *, FrTimeRecorder<GeneralizedVelocity> > m_recorder;    ///< Recorder of the perturbation velocity of the body at COG
    bool c_FScorrection_simple_model = false;
    bool c_FScorrection_extended_model = false;

   public:
    /// Default constructor
    FrRadiationConvolutionModel(const std::string &name,
                                FrOffshoreSystem *system,
                                std::shared_ptr<FrHydroDB> HDB);

    /// Method to initialize the radiation model
    void Initialize() override;

    /// Clear the recorder
    void Clear();

    /// Method to be applied at the end of each time step
    void StepFinalize() override;

    /// Set the impulse response function size
    /// \param BEMBody BEM body database corresponding to the body to which the radiation force is applied
    /// \param Te Time length
    /// \param dt Time step
    void SetImpulseResponseSize(FrBEMBody *BEMBody, double Te, double dt);

    /// Set the impulse response function size
    /// \param body Body to which the radiation force is applied
    /// \param Te Time length
    /// \param dt Time step
    void SetImpulseResponseSize(FrBody *body, double Te, double dt);

    /// Set the impulse response function size
    /// \param Te Time length
    /// \param dt Time step
    void SetImpulseResponseSize(double Te, double dt);

    /// Return the impulse response function size
    /// \param BEMBody BEM body database corresponding to the body to which the radiation force is applied
    /// \param Te Time length
    /// \param dt Time step
    void GetImpulseResponseSize(FrBEMBody *body, double &Te, double &dt) const;

    /// Return the impulse response function size
    /// \param body Body to which the radiation force is applied
    /// \param Te Time length
    /// \param dt Time step
    void GetImpulseResponseSize(FrBody *body, double &Te, double &dt) const;

    /// Setter for activated the forward-speed correction models.
    void ActivateForwardSpeedCorrection(bool activation_simple_model, bool activation_extended_model) {
      c_FScorrection_simple_model = activation_simple_model;
      if(c_FScorrection_simple_model) {
        c_FScorrection_extended_model = activation_extended_model;
      }
    }

   private:

    /// Compute the radiation convolution.
    /// \param time Current time of the simulation from beginning, in seconds
    void Compute(double time) override;

    /// Compute the the convolution part of the radiation force linked with steady speed
    /// \param meanSpeed Steady speed of the body
    /// \return Generalized force
    GeneralizedForce ForwardSpeedCorrection(FrBEMBody *BEMBody) const;

  };

  std::shared_ptr<FrRadiationConvolutionModel>
  make_radiation_convolution_model(const std::string &name, FrOffshoreSystem *system, std::shared_ptr<FrHydroDB> HDB);

}


#endif //FRYDOM_FRRADIATIONCONVOLUTIONMODEL_H
