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

#include "FrRadiationConvolutionModel.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/logging/FrEventLogger.h"
#include "frydom/core/body/FrBody.h"
#include "FrRadiationForce.h"

namespace frydom {

  // TODO : generaliser la méthode de mathutils pour les vecteurs Eigen

  Vector6d<double> TrapzLoc(std::vector<double> &x, std::vector<Vector6d<double>> &y) {

    unsigned long N = y.size();

    //assert(N > 1);
    if (N <= 1) {
      auto res = Vector6d<double>();
      res.SetNull();
      return res;
    }

    assert(x.size() == N);

    double dx1 = x[1] - x[0];
    double dxN_1 = x[N - 1] - x[N - 2];

    Vector6d<double> sum;
    sum.SetNull();
    double dxi, dxii;

    dxii = dx1;

    for (unsigned long i = 1; i < N - 1; i++) {
      dxi = dxii;
      dxii = x[i + 1] - x[i];
      sum += y[i] * (dxi + dxii);
    }

    return 0.5 * (sum + y[0] * dx1 + y[N - 1] * dxN_1);

  };

  FrRadiationConvolutionModel::FrRadiationConvolutionModel(const std::string &name,
                                                           FrOffshoreSystem *system,
                                                           std::shared_ptr<FrHydroDB> HDB) :
      FrRadiationModel(name, system, HDB) {

    /// Construtor of the class.

    // Loop over every body subject to hydrodynamic loads.
    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {
      auto body = m_HDB->GetBody(BEMBody->first);
      body->AddExternalForce(std::make_shared<FrRadiationConvolutionForce>("radiation_force", body, this));
    }
  }

  void FrRadiationConvolutionModel::Initialize() {

    FrRadiationModel::Initialize();

    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {

      // Check if the recorder was instanciated for this BEM body.
      if (m_recorder.find(BEMBody->first) == m_recorder.end()) {
        auto interpK = BEMBody->first->GetIRFInterpolator("K");
        auto Te = interpK->at(0)->GetXmax(BEMBody->first->GetName());
        auto dt = GetParent()->GetTimeStep();
        m_recorder[BEMBody->first] = FrTimeRecorder<GeneralizedVelocity>(Te, dt);
      }
      m_recorder[BEMBody->first].Initialize();
    }
  }

  void FrRadiationConvolutionModel::Clear() {
    for (auto &BEMBody : *m_HDB) {
      m_recorder[BEMBody.first].Clear();
    }
  }

  void FrRadiationConvolutionModel::Compute(double time) {

    if (std::abs(time - GetSystem()->GetTime()) < 0.1 * GetSystem()->GetTimeStep() and
        time > FLT_EPSILON)
      return;

    // Update speed recorder
    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); BEMBody++) {
      auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBody->first);
      m_recorder[BEMBody->first].Record(time, eqFrame->GetPerturbationGeneralizedVelocityInFrame(NWU));
    }

    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {

      auto radiationForce = GeneralizedForce();
      radiationForce.SetNull();

      for (auto BEMBodyMotion = m_HDB->begin(); BEMBodyMotion != m_HDB->end(); ++BEMBodyMotion) {

        auto radiationMask = BEMBody->first->GetRadiationMask(BEMBodyMotion->first);

        auto velocity = m_recorder[BEMBodyMotion->first].GetData();

        auto vtime = m_recorder[BEMBodyMotion->first].GetTime();

        auto BodyMotionDOFMask = m_HDB->GetBodyDOFMask(BEMBodyMotion->first);

        for (auto idof : BodyMotionDOFMask.GetListDOF()) {

          //idof applied here to BEMBodyMotion, even if it's not really explicit. The BEMBodyMotion is now called at the
          // Eval below. So it's equivalent to the next line, previously written for the old container.
          auto interpK = BEMBody->first->GetIRFInterpolator("K")->at(idof);

          std::vector<mathutils::Vector6d<double>> kernel;
          kernel.reserve(vtime.size());
          for (unsigned int it = 0; it < vtime.size(); ++it) {
            auto irf = interpK->Eval(BEMBodyMotion->first->GetName(), vtime[it]) * velocity.at(it).at(idof);
            // radiation mask applied
            kernel.push_back(irf.cwiseProduct(radiationMask.col(idof).cast<double>()));
          }
          radiationForce += TrapzLoc(vtime, kernel);
        }
      }

      auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBody->first);
      auto meanSpeed = eqFrame->GetFrameVelocityInFrame(NWU);

      if (meanSpeed.squaredNorm() > FLT_EPSILON and c_FScorrection_simple_model) {
        radiationForce += ForwardSpeedCorrection(BEMBody->first);
      }

      auto forceInWorld = eqFrame->GetFrame().ProjectVectorFrameInParent(radiationForce.GetForce(), NWU);
      auto TorqueInWorld = eqFrame->GetFrame().ProjectVectorFrameInParent(radiationForce.GetTorque(), NWU);

      // Application of the minus sign.
      m_radiationForce[BEMBody->first] = -GeneralizedForce(forceInWorld, TorqueInWorld);
    }
  }

  void FrRadiationConvolutionModel::StepFinalize() {
    // Serialize and send the message log
//        FrObject::SendLog();
  }

  GeneralizedForce FrRadiationConvolutionModel::ForwardSpeedCorrection(FrBEMBody *BEMBody) const {

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
    if(m_HDB->GetIsXDerivative() and c_FScorrection_extended_model) {
      auto dAdxinf = BEMBody->GetSelfXDerivativeInfiniteAddedMass();
      radiationForce += - meanSpeed.norm() * dAdxinf * eqFrame->GetPerturbationGeneralizedVelocityInFrame(NWU); // -U*dAdx(inf)*V.
    }

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

  void FrRadiationConvolutionModel::SetImpulseResponseSize(FrBEMBody *BEMBody, double Te, double dt) {
    //TODO : check it is not already instanciated
    auto Te_IRF = BEMBody->GetIRFInterpolator("K")->at(0)->GetXmax(BEMBody->GetName());
    if (Te > Te_IRF) {
      event_logger::error("FrRadiationConvolutionModel", GetName(),
                          fmt::format("SetImpulseResponseSize specified Te = {} is larger than the IRF final time = {}.", Te, Te_IRF));
      exit(1);
    }
    m_recorder[BEMBody] = FrTimeRecorder<GeneralizedVelocity>(Te, dt);
  }

  void FrRadiationConvolutionModel::SetImpulseResponseSize(FrBody *body, double Te, double dt) {
    this->SetImpulseResponseSize(m_HDB->GetBody(body), Te, dt);
  }

  void FrRadiationConvolutionModel::SetImpulseResponseSize(double Te, double dt) {
    assert(Te > DBL_EPSILON);
    assert(dt > DBL_EPSILON);
    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {
      this->SetImpulseResponseSize(BEMBody->first, Te, dt);
    }
  }

  void FrRadiationConvolutionModel::GetImpulseResponseSize(FrBEMBody *body, double &Te, double &dt) const {
    Te = m_recorder.at(body).GetTimePersistence();
    dt = m_recorder.at(body).GetTimeStep();
  }

  void FrRadiationConvolutionModel::GetImpulseResponseSize(FrBody *body, double &Te, double &dt) const {
    GetImpulseResponseSize(m_HDB->GetBody(body), Te, dt);
  }

  std::shared_ptr<FrRadiationConvolutionModel>
  make_radiation_convolution_model(const std::string &name,
                                   FrOffshoreSystem *system,
                                   std::shared_ptr<FrHydroDB> HDB) {

    // This subroutine creates and adds the radiation convolution model to the offshore system from the HDB.

    // Construction and initialization of the classes dealing with radiation models.
    auto radiationModel = std::make_shared<FrRadiationConvolutionModel>(name, system, HDB);

    // Addition to the system.
    system->Add(radiationModel);

    return radiationModel;
  }

}