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

#include <frydom/logging/FrEventLogger.h>
#include "FrForce.h"

#include "frydom/asset/FrForceAsset.h"
#include "frydom/logging/FrLogManager.h"
#include "frydom/logging/FrTypeNames.h"


namespace frydom {

  namespace internal {

    FrLoadBodyForceTorqueBase::FrLoadBodyForceTorqueBase(FrForce *force, FrBody* body)
        : chrono::ChLoadCustom(GetChronoBody(body)),
          m_frydomForce(force) {}

    void FrLoadBodyForceTorqueBase::GetBodyForceTorque(Force& force, Torque& torque) const {
      GetForceInWorldNWU(force);
      GetTorqueInBodyNWU(torque);
    }

    void FrLoadBodyForceTorqueBase::GetForceInWorldNWU(Force& force) const {
      force = m_force;
    }

    void FrLoadBodyForceTorqueBase::GetTorqueInBodyNWU(Torque& torque) const {
      auto body = std::dynamic_pointer_cast<chrono::ChBody>(this->loadable);
      torque =  body->TransformDirectionParentToLocal(m_torque).eigen();
    }

    void FrLoadBodyForceTorqueBase::SetForceInWorldNWU(const Force& force) {
      m_force = force;
    }

    void FrLoadBodyForceTorqueBase::SetTorqueInBodyNWU(const Torque& torque) {
      auto body = std::dynamic_pointer_cast<chrono::ChBody>(this->loadable);
      m_torque = body->TransformDirectionLocalToParent(torque).eigen();
    }

    void FrLoadBodyForceTorqueBase::ComputeQ(chrono::ChState* state_x, chrono::ChStateDelta* state_w) {

      auto body = std::dynamic_pointer_cast<chrono::ChBody>(this->loadable);
      if (!body->Variables().IsActive())
        return;

      chrono::ChVectorDynamic<> mF(loadable->Get_field_ncoords());
      mF(0) = m_force.x();
      mF(1) = m_force.y();
      mF(2) = m_force.z();
      mF(3) = m_torque.x();
      mF(4) = m_torque.y();
      mF(5) = m_torque.z();

      // Compute Q = N(u, v, w)'*F
      double detJ; // not used
      auto pos = body->coord.pos; // body abs pos
      body->ComputeNF(pos.x(), pos.y(), pos.z(), load_Q, detJ, mF, state_x, state_w);
    }

    void FrLoadBodyForceTorqueBase::Update(double time) {

      if (!std::dynamic_pointer_cast<chrono::ChBody>(this->loadable)->Variables().IsActive())
        return;

      m_frydomForce->Update(time);

      // Limitation of the force and torque
      if (m_frydomForce->GetLimit()) {

        double limit = m_frydomForce->GetMaxForceLimit();
        double magn = m_force.norm();
        if (magn > limit) {
          m_force *= limit / magn;
        }

        limit = m_frydomForce->GetMaxTorqueLimit();
        magn = m_torque.norm();
        if (magn > limit) {
          m_torque *= limit / magn;
        }
      }

      ChLoadCustom::Update(time);
      ChTime = time;
    }

    std::shared_ptr<FrLoadBodyForceTorqueBase> GetChronoForce(std::shared_ptr<FrForce> force) {
      return force->m_chronoForce;
    }

  }  // end namespace frydom::internal

  // -------------------------------------------------------------------------------
  // FrForce methods implementations
  // -------------------------------------------------------------------------------

  FrForce::FrForce(const std::string &name,
                   const std::string &type_name,
                   FrBody *body) :
      FrLoggable(name, type_name, body) {
      m_chronoForce = std::make_shared<internal::FrLoadBodyForceTorqueBase>(this, body);

  }

  void FrForce::Initialize() {

    if (m_showAsset) {
      m_asset->Initialize();
      GetBody()->AddAsset(m_asset);
    }
  }

  void FrForce::StepFinalize() {}

  void FrForce::DefineLogMessages() {

    auto msg = NewMessage("FrForce", "Force message");

    msg->AddField<double>("Time", "s", "Current time of the simulation",
                          [this]() { return m_chronoForce->GetChTime(); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("ForceInBody", "N", fmt::format("force in body reference frame in {}", GetLogFC()),
         [this]() { return GetForceInBody(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TorqueInBodyAtCOG", "Nm", fmt::format("torque at COG in body reference frame in {}", GetLogFC()),
         [this]() { return GetTorqueInBodyAtCOG(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("ForceInWorld", "N", fmt::format("force in world reference frame in {}", GetLogFC()),
         [this]() { return GetForceInWorld(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TorqueInWorldAtCOG", "Nm", fmt::format("torque at COG in world reference frame in {}", GetLogFC()),
         [this]() { return GetTorqueInWorldAtCOG(GetLogFC()); });

  }

  bool FrForce::IsForceAsset() {
    return m_showAsset;
  }

  void FrForce::ShowAsset(bool isAsset) {
    m_showAsset = isAsset;
    if (isAsset) {
      assert(m_asset == nullptr);
      m_asset = std::make_shared<FrForceAsset>(this);
    }
  }

  void FrForce::SetMaxForceLimit(double fmax) {
    event_logger::info(GetTypeName(), GetName(), "Maximum force limit set to {} N", fmax);
    m_forceLimit = fmax;
  }

  double FrForce::GetMaxForceLimit() const {
    return m_forceLimit;
  }

  void FrForce::SetMaxTorqueLimit(double tmax) {
    event_logger::info(GetTypeName(), GetName(), "Maximum torque limit set to {} N.m", tmax);
    m_torqueLimit = tmax;
  }

  double FrForce::GetMaxTorqueLimit() const {
    return m_torqueLimit;
  }

  void FrForce::SetLimit(bool val) {
    m_limitForce = val;
    if (val) {
      event_logger::info(GetTypeName(), GetName(), "Force/Torque limit activated");
    } else {
      event_logger::info(GetTypeName(), GetName(), "Force/Torque limit deactivated");
    }
  }

  bool FrForce::GetLimit() const {
    return m_limitForce;
  }

  Position FrForce::GetForceApplicationPointInWorld(FRAME_CONVENTION fc) const {
    return GetBody()->GetCOGPositionInWorld(NWU);
  }

  Position FrForce::GetForceApplicationPointInBody(FRAME_CONVENTION fc) const {
    return GetBody()->GetCOG(NWU);
  }

  void FrForce::GetForceInWorld(Force &force, FRAME_CONVENTION fc) const {
    m_chronoForce->GetForceInWorldNWU(force);  // NWU

    if (IsNED(fc)) {
      force = internal::SwapFrameConvention<Force>(force);
    }
  }

  Force FrForce::GetForceInWorld(FRAME_CONVENTION fc) const {
    Force force;
    GetForceInWorld(force, fc);
    return force;
  }

  void FrForce::GetForceInWorld(double &fx, double &fy, double &fz, FRAME_CONVENTION fc) const {
    auto force = GetForceInWorld(fc);
    fx = force[0];
    fy = force[1];
    fz = force[2];
  }

  void FrForce::GetForceInBody(Force &force, FRAME_CONVENTION fc) const {
    GetForceInWorld(force, fc);
    GetBody()->ProjectVectorInBody<Force>(force, fc);
  }

  Force FrForce::GetForceInBody(FRAME_CONVENTION fc) const {
    Force force;
    GetForceInBody(force, fc);
    return force;
  }

  void FrForce::GetForceInBody(double &fx, double &fy, double &fz, FRAME_CONVENTION fc) const {
    auto force = GetForceInBody(fc);
    fx = force[0];
    fy = force[1];
    fz = force[2];
  }

  void FrForce::GetTorqueInWorldAtCOG(Torque &torque, FRAME_CONVENTION fc) const {
    GetTorqueInBodyAtCOG(torque, fc);
    GetBody()->ProjectVectorInWorld<Torque>(torque, fc);
  }

  Torque FrForce::GetTorqueInWorldAtCOG(FRAME_CONVENTION fc) const {
    Torque torque;
    GetTorqueInWorldAtCOG(torque, fc);
    return torque;
  }

  void FrForce::GetTorqueInWorldAtCOG(double &mx, double &my, double &mz, FRAME_CONVENTION fc) const {
    Torque torque = GetTorqueInWorldAtCOG(fc);
    mx = torque[0];
    my = torque[1];
    mz = torque[2];
  }

  void FrForce::GetTorqueInBodyAtCOG(Torque &torque, FRAME_CONVENTION fc) const {
    m_chronoForce->GetTorqueInBodyNWU(torque);

    if (IsNED(fc)) {
      torque = internal::SwapFrameConvention<Torque>(torque);
    }
  }

  Torque FrForce::GetTorqueInBodyAtCOG(FRAME_CONVENTION fc) const {
    Torque torque;
    GetTorqueInBodyAtCOG(torque, fc);
    return torque;
  }

  void FrForce::GetTorqueInBodyAtCOG(double &mx, double &my, double &mz, FRAME_CONVENTION fc) const {
    Torque torque = GetTorqueInBodyAtCOG(fc);
    mx = torque[0];
    my = torque[1];
    mz = torque[2];
  }

  void
  FrForce::GetTorqueInWorldAtPointInWorld(Torque &torque, const Position &worldPoint, FRAME_CONVENTION fc) const {

    Position PG = GetBody()->GetCOGPositionInWorld(fc) - worldPoint;

    torque = GetTorqueInWorldAtCOG(fc) + PG.cross(GetForceInWorld(fc));

  }

  Torque FrForce::GetTorqueInWorldAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {

    Torque torque;
    GetTorqueInWorldAtPointInWorld(torque, worldPoint, fc);
    return torque;

  }

  void FrForce::GetTorqueInWorldAtPointInBody(Torque &torque, const Position &bodyPoint, FRAME_CONVENTION fc) const {

    GetTorqueInWorldAtPointInWorld(torque, GetBody()->GetPointPositionInWorld(bodyPoint, fc), fc);

  }

  Torque FrForce::GetTorqueInWorldAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {

    Torque torque;
    GetTorqueInWorldAtPointInBody(torque, bodyPoint, fc);
    return torque;

  }

  void FrForce::GetTorqueInBodyAtPointInWorld(Torque &torque, const Position &worldPoint, FRAME_CONVENTION fc) const {

    GetTorqueInWorldAtPointInWorld(torque, worldPoint, fc);
    GetBody()->ProjectVectorInBody(torque, fc);

  }

  Torque FrForce::GetTorqueInBodyAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {

    Torque torque;
    GetTorqueInBodyAtPointInWorld(torque, worldPoint, fc);
    return torque;

  }

  void FrForce::GetTorqueInBodyAtPointInBody(Torque &torque, const Position &bodyPoint, FRAME_CONVENTION fc) const {

    GetTorqueInWorldAtPointInBody(torque, bodyPoint, fc);
    GetBody()->ProjectVectorInBody(torque, fc);

  }

  Torque FrForce::GetTorqueInBodyAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {

    Torque torque;
    GetTorqueInBodyAtPointInBody(torque, bodyPoint, fc);
    return torque;

  }


  double FrForce::GetForceNorm() const {
    return GetForceInWorld(NWU).norm();
  }

  double FrForce::GetTorqueNormAtCOG() const {
    return GetTorqueInBodyAtCOG(NWU).norm();
  }

  // =================================================================================================================
  // Protected methods implementations
  // =================================================================================================================

  void FrForce::SetForceInWorldAtCOG(const Force &worldForce, FRAME_CONVENTION fc) {

    /// This subroutine sets a force expressed in the world at the CoG body in Chrono.

    auto forceTmp = worldForce;

    // Transformation if not in NWU.
    if (IsNED(fc)) {
      forceTmp = internal::SwapFrameConvention<Force>(forceTmp);  // In NWU
    }

    m_chronoForce->SetForceInWorldNWU(forceTmp);
  }

  void FrForce::SetForceInWorldAtPointInBody(const Force &worldForce, const Position &bodyPos, FRAME_CONVENTION fc) {
    SetForceInWorldAtCOG(worldForce, fc);

    auto body = GetBody();

    // Calculating the moment created by the force applied at point bodyPos
    Position GP = bodyPos - body->GetCOG(fc); // In body coordinates following the fc convention

    Torque body_torque = GP.cross(body->ProjectVectorInBody<Force>(worldForce, fc));

    SetTorqueInBodyAtCOG(body_torque, fc);
  }

  void
  FrForce::SetForceInWorldAtPointInWorld(const Force &worldForce, const Position &worldPos, FRAME_CONVENTION fc) {
    // Getting the local position of the point
    Position bodyPos = GetBody()->GetPointPositionInBody(worldPos, fc);
    SetForceInWorldAtPointInBody(worldForce, bodyPos, fc);
  }

  void FrForce::SetForceInBody(const Force &bodyForce, FRAME_CONVENTION fc) {
    SetForceInWorldAtCOG(GetBody()->ProjectVectorInWorld<Force>(bodyForce, fc), fc);
  }

  void FrForce::SetForceInBodyAtPointInBody(const Force &bodyForce, const Position &bodyPos, FRAME_CONVENTION fc) {

    SetForceInWorldAtPointInBody(GetBody()->ProjectVectorInWorld<Force>(bodyForce, fc), bodyPos, fc);
  }

  void FrForce::SetForceInBodyAtPointInWorld(const Force &bodyForce, const Position &worldPos, FRAME_CONVENTION fc) {
    SetForceInWorldAtPointInWorld(GetBody()->ProjectVectorInWorld<Force>(bodyForce, fc), worldPos, fc);
  }

  void FrForce::SetTorqueInWorldAtCOG(const Torque &worldTorque, FRAME_CONVENTION fc) {
    SetTorqueInBodyAtCOG(GetBody()->ProjectVectorInBody<Torque>(worldTorque, fc), fc);
  }

  void FrForce::SetTorqueInBodyAtCOG(const Torque &bodyTorque, FRAME_CONVENTION fc) {

    /// This subroutine sets a torque expressed in the world at the CoG body in Chrono.

    auto torqueTmp = bodyTorque;

    // Transformation if not in NWU.
    if (IsNED(fc)) {
      torqueTmp = internal::SwapFrameConvention<Torque>(torqueTmp);  // In NWU
    }

    m_chronoForce->SetTorqueInBodyNWU(torqueTmp);
  }


  void FrForce::SetForceTorqueInWorldAtCOG(const Force &worldForce, const Torque &worldTorque, FRAME_CONVENTION fc) {
    SetForceInWorldAtCOG(worldForce, fc);
    SetTorqueInWorldAtCOG(worldTorque, fc);
  }

  void FrForce::SetForceTorqueInBodyAtCOG(const Force &bodyForce, const Torque &bodyTorque, FRAME_CONVENTION fc) {
    SetForceInBody(bodyForce, fc);
    SetTorqueInBodyAtCOG(bodyTorque, fc);
  }

  void FrForce::SetForceTorqueInWorldAtPointInBody(const Force &worldForce, const Torque &worldTorque,
                                                   const Position &bodyPos, FRAME_CONVENTION fc) {
    SetForceInWorldAtPointInBody(worldForce, bodyPos, fc);
    SetTorqueInBodyAtCOG(GetTorqueInBodyAtCOG(fc) + GetBody()->ProjectVectorInBody<Torque>(worldTorque, fc), fc);
  }

  void FrForce::SetForceTorqueInWorldAtPointInWorld(const Force &worldForce, const Torque &worldTorque,
                                                    const Position &worldPoint, FRAME_CONVENTION fc) {
    SetForceInWorldAtPointInWorld(worldForce, worldPoint, fc);
    SetTorqueInBodyAtCOG(GetTorqueInBodyAtCOG(fc) + GetBody()->ProjectVectorInBody<Torque>(worldTorque, fc), fc);
  }

  void FrForce::SetForceTorqueInBodyAtPointInBody(const Force &bodyForce, const Torque &bodyTorque,
                                                  const Position &bodyPos, FRAME_CONVENTION fc) {
    SetForceInBodyAtPointInBody(bodyForce, bodyPos, fc);
    SetTorqueInBodyAtCOG(GetTorqueInBodyAtCOG(fc) + bodyTorque, fc);
  }

  void FrForce::SetForceTorqueInBodyAtPointInWorld(const Force &bodyForce, const Torque &bodyTorque,
                                                   const Position &worldPos, FRAME_CONVENTION fc) {
    SetForceInBodyAtPointInWorld(bodyForce, worldPos, fc);
    SetTorqueInBodyAtCOG(GetTorqueInBodyAtCOG(fc) + bodyTorque, fc);
  }

  std::shared_ptr<FrForceAsset> FrForce::GetAsset() {
    return m_asset;
  }

  bool FrForce::IsActive() const { return m_isActive; }

  void FrForce::SetActive(bool active) { m_isActive = active; }

  void FrForce::Update(double time) {

    if (IsActive())
      Compute(time);
    else
      SetForceTorqueInBodyAtCOG(Force(), Torque(), NWU);

  }

}  // end namespace frydom
