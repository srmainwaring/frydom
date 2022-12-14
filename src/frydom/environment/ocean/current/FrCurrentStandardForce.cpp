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


#include "FrCurrentStandardForce.h"
#include "frydom/logging/FrTypeNames.h"

#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/core/common/FrRotation.h"
#include "frydom/core/common/FrFrame.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "FrCurrent.h"

namespace frydom {

  FrCurrentStandardForce::FrCurrentStandardForce(const std::string &name, FrBody *body) :
      FrForce(name, TypeToString(this), body) {}

  void FrCurrentStandardForce::SetMaximumBreadth(double breadth) {
    assert(breadth > FLT_EPSILON);
    m_breadth = breadth;
  }

  void FrCurrentStandardForce::SetDraft(double draft) {
    assert(draft > FLT_EPSILON);
    m_draft = draft;
  }

  void FrCurrentStandardForce::SetLateralArea(double lateralArea) {
    assert(lateralArea > FLT_EPSILON);
    m_lateralArea = lateralArea;
  }

  void FrCurrentStandardForce::SetTransverseArea(double transverseArea) {
    assert(transverseArea > FLT_EPSILON);
    m_transverseArea = transverseArea;
  }

  void FrCurrentStandardForce::SetXCenter(double xCenter) {
    m_xCenter = xCenter;
  }

  void FrCurrentStandardForce::SetLengthBetweenPerpendicular(double lpp) {
    assert(lpp > FLT_EPSILON);
    m_lpp = lpp;
  }

  void FrCurrentStandardForce::Initialize() {
    FrForce::Initialize();

    if (m_transverseArea < FLT_EPSILON and m_draft > FLT_EPSILON and m_breadth > FLT_EPSILON) {
      m_transverseArea = m_draft * m_breadth;
    }
    if (m_lateralArea < FLT_EPSILON) throw FrException("error value lateral area");
    if (m_transverseArea < FLT_EPSILON) throw FrException("error value transverse area");
    if (m_lpp < FLT_EPSILON) throw FrException("error value length between perpendicular");
  }

  void FrCurrentStandardForce::Compute(double time) {

    Force force;
    Torque torque;

    auto body = GetBody();

    auto rho = body->GetSystem()->GetEnvironment()->GetOcean()->GetDensity();

    FrFrame FrameAtCOG = body->GetFrameAtCOG();

    auto bodyVelocity = body->GetLinearVelocityInWorld(NWU);
    bodyVelocity.z() = 0.;

    Velocity fluxVelocityInBody =
        body->GetSystem()->GetEnvironment()->GetOcean()->GetCurrent()->GetFluxRelativeVelocityInFrame(FrameAtCOG,
                                                                                                      bodyVelocity, NWU);

    fluxVelocityInBody = internal::SwapFrameConvention(fluxVelocityInBody);
    fluxVelocityInBody = -fluxVelocityInBody;       // Swap convention GOTO/COMEFROM

    double alpha = GetProjectedAngleAroundZ(fluxVelocityInBody, RAD);
    alpha = Normalize_0_2PI(alpha);

    auto ak = 0.5 * rho * fluxVelocityInBody.squaredNorm();

    force.x() = -0.07 * ak * m_transverseArea * cos(alpha);
    force.y() = 0.6 * ak * m_lateralArea * sin(alpha);
    force.z() = 0.;

    if (alpha > M_PI) alpha = 2. * M_PI - alpha;
    auto m1 = std::min(0.4 * (1. - 2. * alpha / M_PI), 0.25);
    auto m2 = std::max(m1, -0.2);
    torque.x() = 0.;
    torque.y() = 0.;
    torque.z() = force.y() * m2 * m_lpp;

    // Build the projected rotation in the XoY plane.
    double phi, theta, psi;
    body->GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, NWU);
    auto bodyRotation = FrRotation(Direction(0., 0., 1.), psi, NWU);
    auto frame = FrFrame(body->GetCOGPositionInWorld(NWU), bodyRotation, NWU);

    auto worldForce = frame.ProjectVectorFrameInParent(force, NWU);
    auto worldTorque = frame.ProjectVectorFrameInParent(torque, NWU);

    SetForceTorqueInWorldAtPointInBody(worldForce, worldTorque, Position(m_xCenter, 0., 0.), NWU);
  }


  std::shared_ptr<FrCurrentStandardForce>
  make_current_standard_force(const std::string &name, std::shared_ptr<FrBody> body) {

    auto force = std::make_shared<FrCurrentStandardForce>(name, body.get());

    body->AddExternalForce(force);

    return force;
  }
}  // end namespace frydom
