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


#include "FrWindStandardForce.h"

#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/atmosphere/FrAtmosphereInc.h"
#include "frydom/logging/FrTypeNames.h"

namespace frydom {

  FrWindStandardForce::FrWindStandardForce(const std::string &name, FrBody *body) :
      FrForce(name, TypeToString(this), body) {}

  void FrWindStandardForce::SetLateralArea(double lateralArea) {
    assert(lateralArea > FLT_EPSILON);
    m_lateralArea = lateralArea;
  }

  void FrWindStandardForce::SetTransverseArea(double transverseArea) {
    assert(transverseArea > FLT_EPSILON);
    m_transverseArea = transverseArea;
  }

  void FrWindStandardForce::SetXCenter(double xCenter) {
    m_xCenter = xCenter;
  }

  void FrWindStandardForce::SetLenghtBetweenPerpendicular(double lpp) {
    assert(lpp > FLT_EPSILON);
    m_lpp = lpp;
  }

  void FrWindStandardForce::Initialize() {
    FrForce::Initialize();
    if (m_transverseArea < FLT_EPSILON) throw FrException(" error value transverse area");
    if (m_lateralArea < FLT_EPSILON) throw FrException("error value lateral area");
    if (m_lpp < FLT_EPSILON) throw FrException("error value length between perpendicular");
  }

  void FrWindStandardForce::Compute(double time) {

    Force force;
    Torque torque;

    auto body = GetBody();
    auto environment = body->GetSystem()->GetEnvironment();

    auto rho = environment->GetAtmosphere()->GetDensity();

    FrFrame FrameAtCOG = body->GetFrameAtCOG();

    auto bodyVelocity = body->GetLinearVelocityInWorld(NWU);
    bodyVelocity.z() = 0.;

    Velocity fluxVelocityInBody = environment->GetAtmosphere()->GetWind()->GetFluxRelativeVelocityInFrame(FrameAtCOG,
                                                                                                          bodyVelocity,
                                                                                                          NWU);

    fluxVelocityInBody = internal::SwapFrameConvention(fluxVelocityInBody);
    fluxVelocityInBody = -fluxVelocityInBody;   // Swap convention GOTO/COMEFROM;

    double alpha = GetProjectedAngleAroundZ(fluxVelocityInBody, RAD);
    alpha = Normalize_0_2PI(alpha);

    auto ak = 0.5 * rho * fluxVelocityInBody.squaredNorm();

    force.x() = -0.7 * ak * m_transverseArea * cos(alpha);
    force.y() = 0.9 * ak * m_lateralArea * sin(alpha);
    force.z() = 0.;

    if (alpha > M_PI) alpha = 2. * M_PI - alpha;
    auto m1 = 0.3 * (1. - 2. * alpha / M_PI);
    torque.x() = 0.;
    torque.y() = 0.;
    torque.z() = force.y() * m1 * m_lpp;

    // Build the projected rotation in the XoY plane.
    double phi, theta, psi;
    body->GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, NWU);
    auto bodyRotation = FrRotation(Direction(0., 0., 1.), psi, NWU);
    auto frame = FrFrame(body->GetCOGPositionInWorld(NWU), bodyRotation, NWU);

    auto worldForce = frame.ProjectVectorFrameInParent(force, NWU);
    auto worldTorque = frame.ProjectVectorFrameInParent(torque, NWU);

    SetForceTorqueInWorldAtPointInBody(worldForce, worldTorque, Position(m_xCenter, 0., 0.), NWU);
  }

  std::shared_ptr<FrWindStandardForce> make_wind_standard_force(const std::string &name, std::shared_ptr<FrBody> body) {

    auto force = std::make_shared<FrWindStandardForce>(name, body.get());

    body->AddExternalForce(force);

    return force;
  }
}  // end namespace frydom
