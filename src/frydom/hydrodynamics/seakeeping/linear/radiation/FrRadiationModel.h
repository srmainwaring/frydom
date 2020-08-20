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

  using RealPoleResiduePair = HDB5_io::RealPoleResiduePair;
  using CCPoleResiduePair = HDB5_io::CCPoleResiduePair;

  /**
   * \class FrRadiationConvolutionModel
   * \brief Class for computing the convolution integrals.
   */
  class FrRadiationRecursiveConvolutionModel : public FrRadiationModel {

   public:
    /// Default constructor
    FrRadiationRecursiveConvolutionModel(const std::string &name,
                                         FrOffshoreSystem *system,
                                         std::shared_ptr<FrHydroDB> HDB);

   private:

//    mathutils::VectorN<double>               c_velocities;
//    mathutils::VectorN<std::complex<double>> c_states;
//    mathutils::VectorN<std::complex<double>> c_alpha;
//    mathutils::VectorN<std::complex<double>> c_beta0;
//    mathutils::VectorN<std::complex<double>> c_beta1;

    Eigen::VectorXcd c_poles;
    Eigen::VectorXcd c_residues;
    Eigen::ArrayXd   c_velocities;
    Eigen::ArrayXcd  c_states;
    Eigen::ArrayXcd  c_alpha;
    Eigen::ArrayXcd  c_beta0;
    Eigen::ArrayXcd  c_beta1;
    double           c_deltaT;
    double           c_time;
    unsigned int     c_N_poles;

    std::unordered_map<FrBEMBody *, GeneralizedVelocity> c_previousVelocity;
//    std::unordered_map<RealPoleResiduePair, double> c_previousRealStates;
//    std::unordered_map<CCPoleResiduePair, std::complex<double>> c_previousCCStates;

    /// Method to initialize the radiation model
    void Initialize() override;


    /// Compute the radiation convolution.
    /// \param time Current time of the simulation from beginning, in seconds
    void Compute(double time) override;

    template<typename T>
    T TrapezoidaleIntegration(T previousState, double velocity, double previousVelocity,
                              HDB5_io::PoleResiduePair<T> poleResiduePair, double DeltaT);

    template<typename T>
    T PiecewiseLinearIntegration(T previousState, double velocity, double previousVelocity,
                                 HDB5_io::PoleResiduePair<T> poleResiduePair, double DeltaT);


    template<typename T>
    mathutils::Vector3d<T> PiecewiseLinearIntegration(mathutils::Vector3d<T> previousStates, double velocity,
                                                      double previousVelocity, mathutils::Vector3d<T> poles,
                                                      double DeltaT);

    void Compute_PieceWiseLinearCoefficients(const Eigen::ArrayXcd& poles, double dt);

    Eigen::ArrayXd GetVelocities() const;

    // TODO:: Add const to FrBEMBody
    GeneralizedForce Compute_RadiationForce(FrBEMBody* body, int &indice) const;

  };


  std::shared_ptr<FrRadiationRecursiveConvolutionModel>
  make_recursive_convolution_model(const std::string &name,
                                   FrOffshoreSystem *system,
                                   std::shared_ptr<FrHydroDB> HDB);
  // -------------------------------------------------------------------------
  // Radiation model with classic convolution
  // -------------------------------------------------------------------------

  /**
   * \class FrRadiationConvolutionModel
   * \brief Class for computing the convolution integrals.
   */
  class FrRadiationConvolutionModel : public FrRadiationModel {

   private:
    std::unordered_map<FrBEMBody *, FrTimeRecorder<GeneralizedVelocity> > m_recorder;    ///< Recorder of the perturbation velocity of the body at COG

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
  make_radiation_convolution_model(const std::string &name,
                                   FrOffshoreSystem *system,
                                   std::shared_ptr<FrHydroDB> HDB);

}  // end namespace frydom


#endif //FRYDOM_FRRADIATIONMODEL_H
