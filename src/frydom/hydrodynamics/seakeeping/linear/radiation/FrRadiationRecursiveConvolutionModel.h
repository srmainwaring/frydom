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

#ifndef FRYDOM_FRRADIATIONRECURSIVECONVOLUTIONMODEL_H
#define FRYDOM_FRRADIATIONRECURSIVECONVOLUTIONMODEL_H

#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrRadiationModel.h"

namespace frydom {

  using RealPoleResiduePair = hdb5_io::RealPoleResiduePair;
  using CCPoleResiduePair = hdb5_io::CCPoleResiduePair;

  /// This class computes the convolution term from the recursive convolution using the pole-residue approximation.
  class FrRadiationRecursiveConvolutionModel : public FrRadiationModel {

   public:
    virtual ~FrRadiationRecursiveConvolutionModel();

    /// Constructor of the class.
    FrRadiationRecursiveConvolutionModel(const std::string &name, FrOffshoreSystem *system, std::shared_ptr<FrHydroDB> HDB);

   protected:

    /// This method computes the recursive convolution..
    void Compute(double time) override;

    /// Method to initialize the radiation model
    void Initialize() override;

   protected:

    // Poles.
    Eigen::VectorXcd c_poles;

    // Residues.
    Eigen::VectorXcd c_residues;

    // Storage of the velocities for the next time step.
    Eigen::ArrayXd c_velocities;

    // Auxiliary variables for the next time step.
    Eigen::ArrayXcd c_states;

    // Parameters for the time integration of the state variables.
    Eigen::ArrayXcd c_alpha;
    Eigen::ArrayXcd c_beta0;
    Eigen::ArrayXcd c_beta1;

    // Time step (s).
    double c_deltaT;

    // Previous time sample (t_{j-1}).
    double c_time;

    // Sum of all vector fitting orders for all coefficients.
    unsigned int c_N_poles;

   private:

    template<typename T>
    T TrapezoidaleIntegration(T previousState, double velocity, double previousVelocity,
                              hdb5_io::PoleResiduePair<T> poleResiduePair, double DeltaT);

    template<typename T>
    T PiecewiseLinearIntegration(T previousState, double velocity, double previousVelocity,
                                 hdb5_io::PoleResiduePair<T> poleResiduePair, double DeltaT);


    template<typename T>
    mathutils::Vector3d<T> PiecewiseLinearIntegration(mathutils::Vector3d<T> previousStates, double velocity,
                                                      double previousVelocity, mathutils::Vector3d<T> poles,
                                                      double DeltaT);

    // This method computes the input parameters for a piecewise linear time-stepping.
    void Compute_PieceWiseLinearCoefficients(const Eigen::ArrayXcd& poles, double dt);

    // This method returns the velocities for every auxiliary variable.
    Eigen::ArrayXd GetVelocities() const;

    /// This method computes the convolution term based on the recursive convolution.
    // TODO:: Add const to FrBEMBody
    GeneralizedForce Compute_RadiationForce(FrBEMBody* body, int &indice) const;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  /// This function creates and adds the radiation recursive convolution model to the offshore system from the HDB.
  std::shared_ptr<FrRadiationRecursiveConvolutionModel>
  make_recursive_convolution_model(const std::string &name, FrOffshoreSystem *system, std::shared_ptr<FrHydroDB> HDB);

}

#endif //FRYDOM_FRRADIATIONRECURSIVECONVOLUTIONMODEL_H
