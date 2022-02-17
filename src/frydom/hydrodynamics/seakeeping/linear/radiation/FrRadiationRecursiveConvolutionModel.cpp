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

#include "FrRadiationRecursiveConvolutionModel.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "FrRadiationForce.h"

namespace frydom {

  FrRadiationRecursiveConvolutionModel::FrRadiationRecursiveConvolutionModel(const std::string &name,
                                                                             FrOffshoreSystem *system,
                                                                             std::shared_ptr<FrHydroDB> HDB) :
      FrRadiationModel(name, system, HDB) {

    // Constructor of the class.

    // Loop over every body subject to hydrodynamic loads.
    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {
      auto body = m_HDB->GetBody(BEMBody->first);
      body->AddExternalForce(std::make_shared<FrRadiationConvolutionForce>("radiation_force", body, this));
    }
  }

  void FrRadiationRecursiveConvolutionModel::Initialize() {

    // Initialization of the Radiation model.
    FrRadiationModel::Initialize();

    // Time step.
    c_deltaT = GetSystem()->GetTimeStep();

    // Previous time sample (t_{j-1}).
    c_time = GetSystem()->GetTime() - c_deltaT; // Useless here as it is the initialization.

    // Sum of all vector fitting orders for all coefficients.
    c_N_poles = 0;

    // Temporary vectors for storing the poles and the residues.
    std::vector<Eigen::VectorXcd> temp_Poles;
    std::vector<Eigen::VectorXcd> temp_Residues;

    // BEMBody force.
    int iBEMBody_force = 0;
    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {

      // BEMBody motion.
      int iBEMBody_motion = 0;
      for (auto BEMBodyMotion = m_HDB->begin(); BEMBodyMotion != m_HDB->end(); ++BEMBodyMotion) {

        auto radiationMask = BEMBody->first->GetRadiationMask(BEMBodyMotion->first);
        auto BodyMotionDOFMask = m_HDB->GetBodyDOFMask(BEMBodyMotion->first);

        // idof of BEMBody motion.
        for (auto idof : BodyMotionDOFMask.GetListDOF()) {

          FrMask radiationMaskForIDOF;
          radiationMaskForIDOF.SetMask(radiationMask.col(idof));

          // iforce of BEMBody force.
          for (auto iforce : radiationMaskForIDOF.GetListDOF()) {

            // TODO : virer les états auxiliaires de la HDB !
            auto poleResidue = BEMBody->first->GetModalCoefficients(BEMBodyMotion->first, idof, iforce);
            auto n_poles = poleResidue.nb_real_poles() + poleResidue.nb_cc_poles();
            c_N_poles += n_poles;
            temp_Poles.push_back(poleResidue.GetPoles());
            temp_Residues.push_back(poleResidue.GetResidues()); // Complex residues are multiplied by 2 here!

          }
        }
        ++iBEMBody_motion;
      }
      ++iBEMBody_force;
    }

    // Initialization of the cached structures.
    c_states.setZero(c_N_poles);
    c_alpha.setZero(c_N_poles);
    c_beta0.setZero(c_N_poles);
    c_beta1.setZero(c_N_poles);
    c_poles.resize(c_N_poles);
    c_residues.resize(c_N_poles);
    c_velocities.setZero(c_N_poles); // The velocities is assumed to be null at t = 0.

    // Storing of the poles.
    int indice = 0;
    for (auto& poles : temp_Poles) {
      auto npoles = poles.size();
      c_poles.segment(indice, npoles) = poles;
      indice += npoles;
    }

    // Storing of the residues.
    indice = 0;
    for (auto& res : temp_Residues) {
      auto npoles = res.size();
      c_residues.segment(indice, npoles) = res;
      indice += npoles;
    }

    // This method computes the input parameters for a piecewise linear time-stepping (alpha, beta, gamma).
    Compute_PieceWiseLinearCoefficients(c_poles, c_deltaT);

  }

  void FrRadiationRecursiveConvolutionModel::Compute(double time) {

    // This method computes the recursive convolution.

    // Time step from the previous time sample.
    auto deltaT = time - c_time; // c_time = t_{j-1}.

    // Present time sample (t_j).
    c_time = time; // c_time = t_j.

    // Computation of the radiation loads only once per time step.
    // The condition of the direct convolution does not work here at the initialization.
    if (deltaT < FLT_EPSILON)
      return;

    // Velocities at the present time sample xdot(t_j).
    auto velocities = GetVelocities();

    if (abs(deltaT - c_deltaT) > 1E-6) {
      c_deltaT = deltaT;
      //TODO : what if poles change?
      Compute_PieceWiseLinearCoefficients(c_poles, deltaT);
    }

    // Computation of u(p, t_j) at the present time sample.
    c_states = c_alpha * c_states + c_beta0 * c_velocities + c_beta1 * velocities;

    // Velocities for the next time step.
    c_velocities = velocities;

    int indice = 0;
    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {

      // Computation of the convolution term from the auxilairy variables.
      auto radiationForce = Compute_RadiationForce(BEMBody->first, indice); // indice is updated here.

      // Setting the convolution term.
      auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBody->first);
      auto forceInWorld = eqFrame->GetFrame().ProjectVectorFrameInParent(radiationForce.GetForce(), NWU);
      auto TorqueInWorld = eqFrame->GetFrame().ProjectVectorFrameInParent(radiationForce.GetTorque(), NWU);

      // Minus sign added.
      m_radiationForce[BEMBody->first] = -GeneralizedForce(forceInWorld, TorqueInWorld);
    }

  }

  GeneralizedForce FrRadiationRecursiveConvolutionModel::Compute_RadiationForce(FrBEMBody* body, int &indice) const {

    // This method computes the convolution term from the auxilairy variables.

    // Initization.
    auto radiationForce = GeneralizedForce();
    radiationForce.SetNull();

    int i_BEMBodyMotion = 0;
    for (auto BEMBodyMotion = m_HDB->begin(); BEMBodyMotion != m_HDB->end(); ++BEMBodyMotion) {

      auto radiationMask = body->GetRadiationMask(BEMBodyMotion->first);
      auto BodyMotionDOFMask = m_HDB->GetBodyDOFMask(BEMBodyMotion->first);

      for (auto idof : BodyMotionDOFMask.GetListDOF()) {

        FrMask radiationMaskForIDOF;
        radiationMaskForIDOF.SetMask(radiationMask.col(idof));

        for (auto iforce : radiationMaskForIDOF.GetListDOF()) {

          // FIXME:: GetModalCoefficients not const
          auto poleResidue = body->GetModalCoefficients(BEMBodyMotion->first, idof, iforce);
          auto n_poles = poleResidue.nb_real_poles() + poleResidue.nb_cc_poles();
          // Contribution of every pole / residue to the convolution term.
          std::complex<double> temp = c_residues.segment(indice, n_poles).transpose() * c_states.matrix().segment(indice, n_poles);

          radiationForce[iforce] += temp.real();

          indice += n_poles;
        }
      }
      ++i_BEMBodyMotion;
    }

    return radiationForce;
  }

  template<typename T>
  T FrRadiationRecursiveConvolutionModel::TrapezoidaleIntegration(T previousState, double velocity,
                                                                  double previousVelocity,
                                                                  hdb5_io::PoleResiduePair<T> poleResiduePair,
                                                                  double DeltaT) {
    auto alpha = exp(DeltaT * poleResiduePair.pole());
    double beta = 0.5 * DeltaT;
    return alpha * previousState + alpha * beta * previousVelocity + beta * velocity;
  }

  template<typename T>
  T FrRadiationRecursiveConvolutionModel::PiecewiseLinearIntegration(T previousState, double velocity,
                                                                     double previousVelocity,
                                                                     hdb5_io::PoleResiduePair<T> poleResiduePair,
                                                                     double DeltaT) {
    auto pole = poleResiduePair.pole();
    auto alpha = exp(DeltaT * pole);
    auto inverseNumerator = 1. / (pole * pole * DeltaT);
    auto beta0 = (1 + (pole * DeltaT - 1) * alpha) * inverseNumerator;
    auto beta1 = (-1 - pole * DeltaT + alpha) * inverseNumerator;
    return alpha * previousState + beta0 * previousVelocity + beta1 * velocity;
  }

  template<typename T>
  Vector3d<T>
  FrRadiationRecursiveConvolutionModel::PiecewiseLinearIntegration(Vector3d<T> previousStates, double velocity,
                                                                   double previousVelocity, Vector3d<T> poles,
                                                                   double DeltaT) {
    //TODO:: check this !
    auto alpha = exp(DeltaT * poles);
    auto inverseNumerator = 1. / (poles * poles * DeltaT);
    auto beta0 = (1 + (poles * DeltaT - 1) * alpha) * inverseNumerator;
    auto beta1 = (-1 - poles * DeltaT + alpha) * inverseNumerator;
    return alpha * previousStates + beta0 * previousVelocity + beta1 * velocity;
  }

  void FrRadiationRecursiveConvolutionModel::Compute_PieceWiseLinearCoefficients(const Eigen::ArrayXcd& poles, double dt) {

    // This method computes the input parameters for a piecewise linear time-stepping.

    assert(poles.size() == c_N_poles);
    auto q2Dt = poles * poles * dt;
    c_alpha = Eigen::exp(poles * dt);
    c_beta0 = (1 + (poles * dt - 1) * c_alpha) / q2Dt.array();
    c_beta1 = (-1 - poles * dt + c_alpha) / q2Dt.array();
  }

  Eigen::ArrayXd FrRadiationRecursiveConvolutionModel::GetVelocities() const {

    // This method returns the velocities for every auxiliary variable.

    Eigen::ArrayXd velocities(c_N_poles);

    int indice = 0;

    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {
      for (auto BEMBodyMotion = m_HDB->begin(); BEMBodyMotion != m_HDB->end(); ++BEMBodyMotion) {
        auto radiationMask = BEMBody->first->GetRadiationMask(BEMBodyMotion->first);
        auto BodyMotionDOFMask = m_HDB->GetBodyDOFMask(BEMBodyMotion->first);

        // Velocity of the body in the equilibrium frame.
        auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBodyMotion->first);
        auto currentVelocity = eqFrame->GetPerturbationGeneralizedVelocityInFrame(NWU);
        for (auto idof : BodyMotionDOFMask.GetListDOF()) {

          FrMask radiationMaskForIDOF;
          radiationMaskForIDOF.SetMask(radiationMask.col(idof));

          //TODO: ne pas calculer les composantes sur les dof fixés de BEMBody?
//          FrMask DOFMask;
//          DOFMask.SetMask(m_HDB->GetMapper()->GetBody(BEMBody->first)->GetDOFMask()->GetFreeDOFs());
//
//          auto BEMBodyMask = radiationMaskForIDOF && DOFMask;
//
//          for (auto iforce : BEMBodyMask.GetListDOF()) {
          for (auto iforce : radiationMaskForIDOF.GetListDOF()) {

            // TODO : virer les états auxiliaires de la HDB !
            auto poleResidue = BEMBody->first->GetModalCoefficients(BEMBodyMotion->first, idof, iforce);
            auto n_poles = poleResidue.nb_real_poles() + poleResidue.nb_cc_poles();
            // The same velocity is used for the all the poles / residues of a coefficient (the VF is vectorized).
            velocities.segment(indice, n_poles) = Eigen::ArrayXd::Constant(n_poles, currentVelocity[idof]);
            indice += n_poles;

          }
        }
      }
    }

    return velocities;
  }

  std::shared_ptr<FrRadiationRecursiveConvolutionModel>
  make_recursive_convolution_model(const std::string &name, FrOffshoreSystem *system, std::shared_ptr<FrHydroDB> HDB) {

    // This function creates and adds the radiation recursive convolution model to the offshore system from the HDB.

    // Construction and initialization of the classes dealing with radiation models.
    auto radiationModel = std::make_shared<FrRadiationRecursiveConvolutionModel>(name, system, HDB);

    // Addition to the system.
    system->Add(radiationModel);

    return radiationModel;
  }

}