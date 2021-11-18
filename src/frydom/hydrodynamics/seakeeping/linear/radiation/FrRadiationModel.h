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


#ifndef FRYDOM_FRRADIATIONMODEL_H
#define FRYDOM_FRRADIATIONMODEL_H

#include <memory>
#include <unordered_map>

#include "frydom/utils/FrRecorder.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrPhysicsItem.h"
#include "frydom/core/common/FrTreeNode.h"

#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"

namespace frydom {

  // Forward declarations
  class FrHydroDB;

//  class FrBEMBody;

  class FrHydroMapper;

  class FrBody;

  class FrOffshoreSystem;

  namespace internal {
    class FrRadiationModelBase;
  }

  /**
   * \class FrRadiationModel
   * \brief Class for computing the radiation loads.
   */
  class FrRadiationModel : public FrTreeNode<FrOffshoreSystem>, public FrPhysicsItem {

   protected:

    std::shared_ptr<FrHydroDB> m_HDB;
    std::unordered_map<FrBEMBody *, GeneralizedForce> m_radiationForce;

   public:

    /// Constructor with specified hydrodynamic database
    /// \param HDB Hydrodynamic database
    explicit FrRadiationModel(const std::string &name,
                              FrOffshoreSystem *system,
                              std::shared_ptr<FrHydroDB> HDB);

    /// Return true if the radiation model is included in the static analysis
    bool IncludedInStaticAnalysis() const override { return false; }

    /// Return the hydrodynamic database linked with the radiation model
    /// \return Hydrodynamic database
    FrHydroDB *GetHydroDB() const { return m_HDB.get(); }

    /// Return the radiation force applied on a body
    /// \param BEMBody BEM body database
    /// \return Radiation force
    Force GetRadiationForce(FrBEMBody *BEMBody) const;

    /// Return the radiation force applied on a body
    /// \param body body (frydom object)
    /// \return Radiation force
    Force GetRadiationForce(FrBody *body) const;

    /// Return the radiation torque applied on a body
    /// \param BEMBody BEM body database
    /// \return Radiation torque
    Torque GetRadiationTorque(FrBEMBody *BEMBody) const;

    /// Return the radiation torque applied on a body
    /// \param body body (frydom object)
    /// \return Radiation torque
    Torque GetRadiationTorque(FrBody *body) const;

    /// Return the generalized force part relative to the added mass term
    /// \param body Body for which the motion is considered
    /// \return Part the the radiation force linked with the acceleration of the body
    GeneralizedForce GetRadiationInertiaPart(FrBody *body) const;

    /// Method to initialize the radiation model
    void Initialize() override;

    /// Return the mapper between body and BEM body database
    /// \return Mapper
    FrHydroMapper *GetMapper() const;

   private:

    /// Compute the internal states of the Radiation model
    /// \param time Current time of the simulation from beginning, in seconds
    void Compute(double time) override;

  };


  // -------------------------------------------------------------------------
  // Radiation model with recursive convolution
  // -------------------------------------------------------------------------

  using RealPoleResiduePair = hdb5_io::RealPoleResiduePair;
  using CCPoleResiduePair = hdb5_io::CCPoleResiduePair;

  /**
   * \class FrRadiationConvolutionModel
   * \brief Class for computing the convolution integrals.
   */
  class FrRadiationRecursiveConvolutionModel : public FrRadiationModel {

   public:

    /// Constructor of the class.
    FrRadiationRecursiveConvolutionModel(const std::string &name, FrOffshoreSystem *system, std::shared_ptr<FrHydroDB> HDB);

    /// Setter for activated the forward-speed correction model.
    void ActivateForwardSpeedCorrection(bool activation_simple_model) {
      c_FScorrection_simple_model = activation_simple_model;
    }

   private:

    // Poles.
    Eigen::VectorXcd c_poles;

    // Residues.
    Eigen::VectorXcd c_residues;

    // Storage of the velocities for the next time step.
    Eigen::ArrayXd c_velocities;
    Eigen::ArrayXd c_velocities_forward_speed;

    // Auxiliary variables for the next time step.
    Eigen::ArrayXcd c_states;
    Eigen::ArrayXcd c_states_forward_speed; // Application of the matrix L on the velocity.

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

    // Simple forward speed model.
    bool c_FScorrection_simple_model = false;

    // Matrix to know the index of every radiation coefficient in the cached vectors of size c_N_poles.
    mathutils::MatrixMN<double> c_matrix_index;

    // Residues over the poles.
    Eigen::VectorXcd c_residues_over_poles;

    // Storage of the initial positions.
    std::vector<mathutils::Vector6d<double>> c_initial_positions;

    /// Method to initialize the radiation model
    void Initialize() override;

    /// This method computes the recursive convolution..
    void Compute(double time) override;

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

    // This method returns the velocities impacted by the matrix L for the forward speed for every auxiliary variable.
    Eigen::ArrayXd GetVelocitiesForwardSpeed() const;

    // This method returns the positions in world of all bodies.
    std::vector<mathutils::Vector6d<double>> GetPositions() const;

    // TODO:: Add const to FrBEMBody
    GeneralizedForce Compute_RadiationForce(FrBEMBody* body, int &indice) const;

  };


  std::shared_ptr<FrRadiationRecursiveConvolutionModel>
  make_recursive_convolution_model(const std::string &name,
                                   FrOffshoreSystem *system,
                                   std::shared_ptr<FrHydroDB> HDB);

}  // end namespace frydom


#endif //FRYDOM_FRRADIATIONMODEL_H
