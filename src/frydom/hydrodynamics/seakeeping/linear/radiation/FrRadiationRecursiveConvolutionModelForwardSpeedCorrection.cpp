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

#include "FrRadiationRecursiveConvolutionModelForwardSpeedCorrection.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"

namespace frydom {

  FrRadiationRecursiveConvolutionModelForwardSpeedCorrection::FrRadiationRecursiveConvolutionModelForwardSpeedCorrection(
      const std::string &name, FrOffshoreSystem *system, std::shared_ptr<FrHydroDB> HDB) :
      FrRadiationRecursiveConvolutionModel(name, system, HDB) {

    // Constructor of the class.

  }

  void FrRadiationRecursiveConvolutionModelForwardSpeedCorrection::Initialize() {

    // This methods initializes the recursive radiation model with forward speed correction.

    // Initialization of the zero forward speed recursive convolution model.
    FrRadiationRecursiveConvolutionModel::Initialize();

    // Matrix for knowing the index of every radiation coefficient in the cached vectors of size c_N_poles.
    int nbBEMBody = m_HDB->GetBEMBodyNumber();
    c_matrix_index = mathutils::MatrixMN<double>::Zero(6 * nbBEMBody, 6 * nbBEMBody);
    int N_poles = 0;
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
            c_matrix_index(6 * iBEMBody_force + iforce, 6 * iBEMBody_motion + idof) = N_poles;
            N_poles += n_poles;

          }
        }
        ++iBEMBody_motion;
      }
      ++iBEMBody_force;
    }

    // Initialization of the cached structures.
    c_states_forward_speed.setZero(c_N_poles);
    c_velocities_forward_speed.setZero(c_N_poles); // The velocities is assumed to be null at t = 0.

    // Extra cached quantities for the forward speed correction.
    c_residues_over_poles.setZero(c_N_poles);
    for (int i = 0; i < c_residues_over_poles.size(); ++i){
      c_residues_over_poles(i) = c_residues(i) / c_poles(i);
    }
    c_initial_positions = GetPositions();

  }

  void FrRadiationRecursiveConvolutionModelForwardSpeedCorrection::Compute(double time) {

    // This method computes the radiation loads with the forward speed correction.

    // Computation of the radiation loads only once per time step.
    if (std::abs(time - GetSystem()->GetTime()) < 0.1 * GetSystem()->GetTimeStep() and
        time > FLT_EPSILON)
      return;

    // Computation of the zero forward speed recursive convolution term.
    FrRadiationRecursiveConvolutionModel::Compute(time);

    // Velocities at the present time sample xdot(t_j).
    auto velocities_forward_speed = GetVelocitiesForwardSpeed();

    // Computation of u(p, t_j) at the present time sample.
    c_states_forward_speed = c_alpha * c_states_forward_speed + c_beta0 * c_velocities_forward_speed + c_beta1 * velocities_forward_speed;

    // Velocities for the next time step.
    c_velocities_forward_speed = velocities_forward_speed;

    int i_body = 0;
    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {

      // Computation of the convolution term from the auxilairy variables.
      auto forward_speed_correction = ForwardSpeedCorrection(BEMBody->first, i_body);

      // Setting the convolution term.
      auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBody->first);
      auto forceInWorld = eqFrame->GetFrame().ProjectVectorFrameInParent(forward_speed_correction.GetForce(), NWU);
      auto TorqueInWorld = eqFrame->GetFrame().ProjectVectorFrameInParent(forward_speed_correction.GetTorque(), NWU);

      // Minus sign added.
      m_radiationForce[BEMBody->first] += -GeneralizedForce(forceInWorld, TorqueInWorld);

      ++i_body;
    }

  }

  GeneralizedForce FrRadiationRecursiveConvolutionModelForwardSpeedCorrection::ForwardSpeedCorrection(FrBEMBody *body,
                                                                                                      int &i_body) const {

    // This method computes the the forward speed dependent part of the convolution term.

    auto SpeedCorrection = GeneralizedForce();
    SpeedCorrection.setZero();
    auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(body);
    auto meanSpeed = eqFrame->GetFrameVelocityInFrame(NWU);

    // Positions.
    auto positions = GetPositions();

    //WARNING: Only the influence of the body on itself is used for the forward speed model.

    // Radiation mask.
    auto radiationMask = body->GetRadiationMask(body);
    auto BodyMotionDOFMask = m_HDB->GetBodyDOFMask(body);

    for (auto idof : BodyMotionDOFMask.GetListDOF()) {

      FrMask radiationMaskForIDOF;
      radiationMaskForIDOF.SetMask(radiationMask.col(idof));

      for (auto iforce : radiationMaskForIDOF.GetListDOF()) {

        if(idof >= 4) { // Application of the matrix L.

          // Application of the matrix L for selecting the poles and residues.
          double epsilon = 1.;
          hdb5_io::PoleResidue poleResidue;
          int idof_coupling;
          if(idof == 4){ // Pitch.
            epsilon = -1.;
            idof_coupling = 2;
          } else { // idof = 5 (yaw).
            idof_coupling = 1;
          }
          poleResidue = body->GetModalCoefficients(body, idof_coupling, iforce);
          auto indice_forward_speed = c_matrix_index(6 * i_body + iforce, 6 * i_body + idof_coupling);
          auto n_poles = poleResidue.nb_real_poles() + poleResidue.nb_cc_poles();

          // Because of the radiation mask, the number of poles may be zero?
          if(n_poles != 0) {

            // Recursive convolution.
            auto residues_over_poles = c_residues_over_poles.segment(indice_forward_speed, n_poles);
            auto sum_residues_over_poles = residues_over_poles.sum();
            double position_dof = positions.at(i_body)(idof);

            std::complex<double> speed_correction = -residues_over_poles.transpose() * c_states_forward_speed.matrix().segment(indice_forward_speed, n_poles);
            speed_correction += (position_dof - c_initial_positions.at(i_body)(idof)) * sum_residues_over_poles;
            SpeedCorrection[iforce] += meanSpeed.norm() * epsilon * speed_correction.real();

          }
        }
      }
    }

    // Infinite frequency damping.
    auto angular = eqFrame->GetPerturbationAngularVelocityInFrame(NWU); // Angular velocity.
    auto Ainf = body->GetSelfInfiniteAddedMass(); // Infinite frequency added mass.
    auto damping = Ainf.col(2) * angular.y() - Ainf.col(1) * angular.z(); // -A(inf)*L*V.
    SpeedCorrection += meanSpeed.norm() * damping; // -U*A(inf)*L*V.

    return SpeedCorrection;

  }


  Eigen::ArrayXd FrRadiationRecursiveConvolutionModelForwardSpeedCorrection::GetVelocitiesForwardSpeed() const {

    // This method returns the velocities for every auxiliary variable.

    Eigen::ArrayXd velocities(c_N_poles);

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

          for (auto iforce : radiationMaskForIDOF.GetListDOF()) {

            if(idof >= 4) { // Application of the matrix L.
              int idof_coupling;
              if (idof == 4) { // Pitch.
                idof_coupling = 2;
              } else { // idof = 5 (yaw).
                idof_coupling = 1;
              }

              auto poleResidue = BEMBody->first->GetModalCoefficients(BEMBodyMotion->first, idof_coupling, iforce);
              auto n_poles = poleResidue.nb_real_poles() + poleResidue.nb_cc_poles();
              auto indice = c_matrix_index(iforce, idof_coupling);

              // The same velocity is used for the all the poles / residues of a coefficient (the VF is vectorized).
              velocities.segment(indice, n_poles) = Eigen::ArrayXd::Constant(n_poles, currentVelocity[idof]);

            }
          }
        }
      }
    }

    return velocities;
  }

  std::vector<mathutils::Vector6d<double>> FrRadiationRecursiveConvolutionModelForwardSpeedCorrection::GetPositions() const {

    // This method returns the positions in world of all bodies.

    std::vector<mathutils::Vector6d<double>> positions;
    positions.reserve(m_HDB->GetBEMBodyNumber());
    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {

      auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBody->first);
      auto position = eqFrame->GetPositionInWorld(NWU);
      double phi, theta, psi;
      eqFrame->GetPerturbationFrame().GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, NWU);
      positions.push_back(Vector6d(position(0), position(1), position(2), phi, theta, psi));
    }

    return positions;
  }

  std::shared_ptr<FrRadiationRecursiveConvolutionModelForwardSpeedCorrection>
  make_recursive_convolution_model_with_forward_speed_correction(const std::string &name, FrOffshoreSystem *system, std::shared_ptr<FrHydroDB> HDB) {

    // This function creates and adds the radiation recursive convolution model with forward speed to the offshore system from the HDB.

    // Construction and initialization of the classes dealing with radiation models.
    auto radiationModel = std::make_shared<FrRadiationRecursiveConvolutionModelForwardSpeedCorrection>(name, system, HDB);

    // Addition to the system.
    system->Add(radiationModel);

    return radiationModel;
  }

}