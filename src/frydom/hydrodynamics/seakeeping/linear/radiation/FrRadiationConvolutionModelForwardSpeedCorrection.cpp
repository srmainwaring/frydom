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

#include "FrRadiationConvolutionModelForwardSpeedCorrection.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"

namespace frydom {

  FrRadiationConvolutionModelForwardSpeedCorrection::FrRadiationConvolutionModelForwardSpeedCorrection(
      const std::string &name, FrOffshoreSystem *system, std::shared_ptr<FrHydroDB> HDB) :
      FrRadiationConvolutionModel(name, system, HDB) {

    // Constructor of the class.

  }

  void FrRadiationConvolutionModelForwardSpeedCorrection::Compute(double time) {

    // This method computes the radiation loads with the forward speed correction.

    // Computation of the radiation only once per time step.
    if (std::abs(time - GetSystem()->GetTime()) < 0.1 * GetSystem()->GetTimeStep() and
        time > FLT_EPSILON)
      return;

    // Computation of the zero forward speed convolution term.
    FrRadiationConvolutionModel::Compute(time);

    // Forward-speed correction.
    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {
      auto forward_speed_correction = ForwardSpeedCorrection(BEMBody->first);

      auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBody->first);
      auto forceInWorld = eqFrame->GetFrame().ProjectVectorFrameInParent(forward_speed_correction.GetForce(), NWU);
      auto TorqueInWorld = eqFrame->GetFrame().ProjectVectorFrameInParent(forward_speed_correction.GetTorque(), NWU);

      // Application of the minus sign.
      m_radiationForce[BEMBody->first] += -GeneralizedForce(forceInWorld, TorqueInWorld);

    }

  }

  GeneralizedForce FrRadiationConvolutionModelForwardSpeedCorrection::ForwardSpeedCorrection(FrBEMBody *BEMBody) const {

    // This method computes the forward speed correction of the radiation loads.

    //WARNING: Only the influence of the body on itself is used for the forward speed model.

    //TODO: Il faudrait ajouter des termes de raideur dependant de la vitesse d'avance.

    // Initialization.
    auto radiationForce = GeneralizedForce();
    radiationForce.SetNull();

    // Forward speed.
    auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBody);
    auto meanSpeed = eqFrame->GetFrameVelocityInFrame(NWU);

    // Angular velocity.
    auto angular = eqFrame->GetPerturbationAngularVelocityInFrame(NWU);

    // Infinite frequency added mass.
    auto Ainf = BEMBody->GetSelfInfiniteAddedMass();

    // Stored body velocity for the convolution.
    auto velocity = m_recorder.at(BEMBody).GetData();

    // Stored time for the convolution.
    auto vtime = m_recorder.at(BEMBody).GetTime();

    // KU.
    for (unsigned int idof = 4; idof < 6; idof++) { // Pitch and yaw only.
      // Convolution.
      auto interpKu = BEMBody->GetIRFInterpolator("KU")->at(idof); // The matrix L has already been applied in hdb5tool.
      std::vector<mathutils::Vector6d<double>> kernel;
      for (unsigned int it = 0; it < vtime.size(); ++it) {
        kernel.push_back(interpKu->Eval(BEMBody->GetName(), vtime[it]) * velocity[it].at(idof)); // int(KU(t-tau)*L*v(tau)).
      }
      radiationForce += meanSpeed.norm() * TrapzLoc(vtime, kernel); // U*int(KU(t-tau)*L*v(tau)).
    }

    // KUXDerivative.
//    if(m_HDB->GetIsXDerivative() and c_FScorrection_extended_model) {
//      auto KUXderivativeForce = GeneralizedForce();
//      KUXderivativeForce.SetNull();
//      auto KU2Force = GeneralizedForce();
//      KU2Force.SetNull();
//      for (unsigned int idof = 0; idof < 6; idof++) {
//        // Convolution.
//        auto interpKuXderivative = BEMBody->GetIRFInterpolator("KUXDerivative")->at(idof);
//        auto interpKu2 = BEMBody->GetIRFInterpolator("KU2")->at(idof);
//        std::vector<mathutils::Vector6d<double>> kernel_KUXDerivative;
//        std::vector<mathutils::Vector6d<double>> kernel_KU2;
//        for (unsigned int it = 0; it < vtime.size(); ++it) {
//          kernel_KUXDerivative.push_back(interpKuXderivative->Eval(BEMBody->GetName(), vtime[it]) * velocity[it].at(idof)); // KUXderivative(t-tau)*v(tau).
//          if(idof == 4 or idof == 5){
//            kernel_KU2.push_back(interpKu2->Eval(BEMBody->GetName(), vtime[it]) * velocity[it].at(idof)); // KU2(t-tau)*v(tau).
//          }
//        }
//        KUXderivativeForce += TrapzLoc(vtime, kernel_KUXDerivative) * meanSpeed.norm();
//        if(idof == 4 or idof == 5) {
//          KU2Force += TrapzLoc(vtime, kernel_KU2) * meanSpeed.norm() * meanSpeed.norm();
//        }
//      }
//      radiationForce += KUXderivativeForce + KU2Force;
////      radiationForce += -(KUXderivativeForce + KU2Force); // Minus sign due to transformation from the body frame to the world frame.
//    }

    // Infinite frequency damping.
    auto damping = Ainf.col(2) * angular.y() - Ainf.col(1) * angular.z(); // -A(inf)*L*V.
    radiationForce += meanSpeed.norm() * damping; // -U*A(inf)*L*V.
//    if(m_HDB->GetIsXDerivative()) {
//      auto dAdxinf = BEMBody->GetSelfXDerivativeInfiniteAddedMass();
//      radiationForce += - meanSpeed.norm() * dAdxinf * eqFrame->GetPerturbationGeneralizedVelocityInFrame(NWU); // -U*dAdx(inf)*V.
//    }

    // Speed-dependent stiffness.
//    if(m_HDB->GetIsXDerivative() and c_FScorrection_extended_model) {
//      double phi, theta, psi;
//      eqFrame->GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, NWU);
//      CardanAngles angular_position(phi, theta, psi); // Angular position of the body frame with respect to the equilibrium frame expressed in the equilibrium frame.
//      auto dAdx0 = BEMBody->GetSelfXDerivativeZeroAddedMass(); // dAdx(0).
//      auto stiffness = -dAdx0.col(2) * angular_position.GetPitch() + dAdx0.col(1) * angular_position.GetYaw(); // dAdx(0)*L*X.
//      radiationForce += meanSpeed.norm() * meanSpeed.norm() * stiffness; // U^2*dAdx(0)*L*X.
//    }

    return radiationForce;

  }

  std::shared_ptr<FrRadiationConvolutionModel>
  make_radiation_convolution_model_with_forward_speed_correction(const std::string &name,
                                                                 FrOffshoreSystem *system,
                                                                 std::shared_ptr<FrHydroDB> HDB) {

    // This method creates and adds the radiation convolution model with forward speed correction to the offshore system from the HDB.

    // Construction and initialization of the classes dealing with radiation models.
    auto radiationModel = std::make_shared<FrRadiationConvolutionModelForwardSpeedCorrection>(name, system, HDB);

    // Addition to the system.
    system->Add(radiationModel);

    return radiationModel;
  }

}