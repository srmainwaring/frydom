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


#include "FrRadiationForce.h"

#include "FrRadiationModel.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/logging/FrTypeNames.h"


namespace frydom {

  // --------------------------------------------------
  // FrRadiationForce
  // --------------------------------------------------

  FrRadiationForce::FrRadiationForce(const std::string &name,
                                     FrBody *body,
                                     FrRadiationModel *radiationModel)
      : FrForce(name, TypeToString(this), body),
        m_radiationModel(radiationModel) {}

  void FrRadiationForce::SetRadiationModel(FrRadiationModel *radiationModel) {
    m_radiationModel = radiationModel;
  }

  // --------------------------------------------------
  // FrRadiationConvolutionForce
  // --------------------------------------------------

  FrRadiationConvolutionForce::FrRadiationConvolutionForce(const std::string &name,
                                                           FrBody *body,
                                                           FrRadiationModel *radiationModel)
      : FrRadiationForce(name, body, radiationModel) {}

  void FrRadiationConvolutionForce::Initialize() {
    FrRadiationForce::Initialize();
  }

  void FrRadiationConvolutionForce::StepFinalize() {
    this->UpdateForceInertiaPart();
    FrRadiationForce::StepFinalize();
  }

  void FrRadiationConvolutionForce::Compute(double time) {

    auto body = GetBody();

    auto force = m_radiationModel->GetRadiationForce(body);
    auto torque = m_radiationModel->GetRadiationTorque(body);

    //##CC
    auto inertia_part = m_radiationModel->GetRadiationSteadyInertiaPart(body);
    force += inertia_part.GetForce();
    torque += inertia_part.GetTorque();
    //##

    SetForceTorqueInWorldAtCOG(force, torque, NWU);

    this->UpdateForceInertiaPart();
  }

  void FrRadiationConvolutionForce::UpdateForceInertiaPart() {

    auto forceInertiaPart = m_radiationModel->GetRadiationInertiaPart(GetBody());
    c_forceInertiaPart = forceInertiaPart.GetForce();
    c_torqueInertiaPart = forceInertiaPart.GetTorque();
  }

  Force FrRadiationConvolutionForce::GetForceInertiaPartInBody(FRAME_CONVENTION fc) const {
    auto force = c_forceInertiaPart;
    if (IsNED(fc)) { force = internal::SwapFrameConvention<Force>(force); }
    return force;
  }

  Torque FrRadiationConvolutionForce::GetTorqueInertiaPartInBody(FRAME_CONVENTION fc) const {
    auto torque = c_torqueInertiaPart;
    if (IsNED(fc)) { torque = internal::SwapFrameConvention<Torque>(torque); }
    return torque;
  }

  Force FrRadiationConvolutionForce::GetForceInertiaPartInWorld(FRAME_CONVENTION fc) const {
    auto force = GetBody()->ProjectVectorInWorld<Force>(c_forceInertiaPart, fc);
    return force;
  }

  Torque FrRadiationConvolutionForce::GetTorqueInertiaPartInWorld(FRAME_CONVENTION fc) const {
    auto torque = GetBody()->ProjectVectorInWorld(c_torqueInertiaPart, fc);
    return torque;
  }

  void FrRadiationConvolutionForce::DefineLogMessages() {

    auto msg = NewMessage("FrRadiationConvolutionForce", "Force message");

    msg->AddField<double>("Time", "s", "Current time of the simulation",
                          [this]() { return m_chronoForce->GetChTime(); });

    // Log of the convolution part of the force
    msg->AddField<Eigen::Matrix<double, 3, 1>>
                    ("ForceConvolutionInBody","N", fmt::format("Convolution part of the force in body reference frame in {}", GetLogFC()),
                     [this]() {return GetForceInBody(GetLogFC());});

    msg->AddField<Eigen::Matrix<double, 3, 1>>
                    ("TorqueConvolutionInBodyAtCOG","Nm", fmt::format("Convolution part of the torque at COG in body reference frame in {}", GetLogFC()),
                     [this]() {return GetTorqueInBodyAtCOG(GetLogFC());});

    msg->AddField<Eigen::Matrix<double, 3, 1>>
                    ("ForceConvolutionInWorld","N", fmt::format("Convolution part of the force in world reference frame in {}", GetLogFC()),
                     [this]() {return GetForceInWorld(GetLogFC());});

    msg->AddField<Eigen::Matrix<double, 3, 1>>
                    ("TorqueConvolutionInWorldAtCOG","Nm", fmt::format("Convolution path of the torque at COG in world reference frame in {}", GetLogFC()),
                     [this]() {return GetTorqueInWorldAtCOG(GetLogFC());});

            // Log the inertia part of the force
    msg->AddField<Eigen::Matrix<double, 3, 1>>
                    ("ForceInertiaInBody", "N", fmt::format("Inertia part of the force in body reference frame in {}", GetLogFC()),
                     [this]() {return GetForceInertiaPartInBody(GetLogFC());});

    msg->AddField<Eigen::Matrix<double, 3, 1>>
                    ("TorqueInertiaInBodyAtCOG", "N.m", fmt::format("Inertia part of the force in body reference frame in {}", GetLogFC()),
                     [this]() {return GetTorqueInertiaPartInBody(GetLogFC());});

    msg->AddField<Eigen::Matrix<double, 3, 1>>
                    ("ForceInertiaInWorld", "N", fmt::format("Inertia part of the force in world reference frame in {}", GetLogFC()),
                     [this]() {return GetForceInertiaPartInWorld(GetLogFC());});

    msg->AddField<Eigen::Matrix<double, 3, 1>>
                    ("TorqueInertiaInWorldAtCOG", "Nm", fmt::format("Inertia part of the torque at COG in world reference frame in {}", GetLogFC()),
                     [this]() {return GetTorqueInertiaPartInWorld(GetLogFC());});

            // Log the whole radiation force
    msg->AddField<Eigen::Matrix<double, 3, 1>>
                    ("ForceInBody","N", fmt::format("Convolution part of the force in body reference frame in {}", GetLogFC()),
                     [this]() {return GetForceInBody(GetLogFC()) + GetForceInertiaPartInBody(GetLogFC());});

    msg->AddField<Eigen::Matrix<double, 3, 1>>
                    ("TorqueInBodyAtCOG","Nm", fmt::format("Convolution part of the torque at COG in body reference frame in {}", GetLogFC()),
                     [this]() {return GetTorqueInBodyAtCOG(GetLogFC()) + GetTorqueInertiaPartInBody(GetLogFC());});

    msg->AddField<Eigen::Matrix<double, 3, 1>>
                    ("ForceInWorld","N", fmt::format("Convolution part of the force in world reference frame in {}", GetLogFC()),
                     [this]() {return GetForceInWorld(GetLogFC()) + GetForceInertiaPartInWorld(GetLogFC());});

    msg->AddField<Eigen::Matrix<double, 3, 1>>
                    ("TorqueInWorldAtCOG","Nm", fmt::format("Convolution path of the torque at COG in world reference frame in {}", GetLogFC()),
                     [this]() {return GetTorqueInWorldAtCOG(GetLogFC()) + GetTorqueInertiaPartInWorld(GetLogFC());});

  }

}  // end namespace frydom
