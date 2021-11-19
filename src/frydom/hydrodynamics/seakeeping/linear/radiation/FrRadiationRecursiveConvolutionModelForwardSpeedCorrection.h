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

#ifndef FRYDOM_FRRADIATIONRECURSIVECONVOLUTIONMODELFORWARDSPEEDCORRECTION_H
#define FRYDOM_FRRADIATIONRECURSIVECONVOLUTIONMODELFORWARDSPEEDCORRECTION_H

#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrRadiationRecursiveConvolutionModel.h"

namespace frydom {

  /// This class computes the forward speed correction of the convolution term from the recursive convolution using the pole-residue approximation.
  class FrRadiationRecursiveConvolutionModelForwardSpeedCorrection : public FrRadiationRecursiveConvolutionModel{

   public:

    /// Constructor of the class.
    FrRadiationRecursiveConvolutionModelForwardSpeedCorrection(const std::string &name, FrOffshoreSystem *system, std::shared_ptr<FrHydroDB> HDB);

   private:

    /// This methods initializes the recursive radiation model with forward speed correction.
    void Initialize() override;

    /// This method computes the radiation loads with the forward speed correction.
    void Compute(double time) override;

    /// This method computes the the forward speed dependent part of the convolution term.
    GeneralizedForce ForwardSpeedCorrection(FrBEMBody *body, int &i_body) const;

    // This method returns the velocities impacted by the matrix L for the forward speed for every auxiliary variable.
    Eigen::ArrayXd GetVelocitiesForwardSpeed() const;

    // This method returns the positions in world of all bodies.
    std::vector<mathutils::Vector6d<double>> GetPositions() const;

   private:

    // Storage of the velocities for the next time step.
    Eigen::ArrayXd c_velocities_forward_speed; // Application of the matrix L on the velocity.

    // Auxiliary variables for the next time step.
    Eigen::ArrayXcd c_states_forward_speed; // Application of the matrix L on the velocity.

    // Matrix to know the index of every radiation coefficient in the cached vectors of size c_N_poles.
    mathutils::MatrixMN<double> c_matrix_index;

    // Residues over the poles.
    Eigen::VectorXcd c_residues_over_poles;

    // Storage of the initial positions.
    std::vector<mathutils::Vector6d<double>> c_initial_positions;

  };

  /// This function creates and adds the radiation recursive convolution model with forward speed to the offshore system from the HDB.
  std::shared_ptr<FrRadiationRecursiveConvolutionModelForwardSpeedCorrection>
  make_recursive_convolution_model_with_forward_speed_correction(const std::string &name, FrOffshoreSystem *system, std::shared_ptr<FrHydroDB> HDB);

}

#endif //FRYDOM_FRRADIATIONRECURSIVECONVOLUTIONMODELFORWARDSPEEDCORRECTION_H
