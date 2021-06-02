//
// Created by lletourn on 21/05/2021.
//

#include "FrPropulsionForce.h"
#include "frydom/environment/FrEnvironmentInc.h"
#include "frydom/core/body/FrBody.h"

namespace frydom {

  FrPropulsionForce::FrPropulsionForce(const std::string& name, FrBody *body, double thrustDeductionFactor, double LOA, double ku):
  FrForce(name, "FrPropulsionName", body), m_shipLOA(LOA), m_thrust_deduction_factor(thrustDeductionFactor),
  m_K1(4), m_wake_fraction0(0.4), m_correction_factor(ku), m_longitudinal_velocity(0.), m_transversal_velocity(0.){

  }


  void FrPropulsionForce::ComputeLongitudinalVelocity() {
    auto body = GetBody();
    auto propFrame = body->GetFrameAtPoint(m_positionInBody, NWU);
    auto propVelocityInWorld = body->GetVelocityInWorldAtPointInBody(m_positionInBody, NWU);
    auto uP0 = body->GetSystem()->GetEnvironment()->GetRelativeVelocityInFrame(propFrame, propVelocityInWorld, WATER, NWU).x();
    m_longitudinal_velocity = m_correction_factor * uP0 * (1.-GetWakeFraction());
  }

  double FrPropulsionForce::GetSideWashAngle() const {
    return GetBody()->GetVelocityInHeadingFrame(NWU).GetProjectedAngleAroundZ(RAD);
  }

  double FrPropulsionForce::GetWakeFraction() const {
    auto gamma = GetSideWashAngle();
    return m_wake_fraction0 * exp(-m_K1 * gamma * gamma);
  }

  void FrPropulsionForce::SetStraightRunWakeFraction(double wp0) {
    m_wake_fraction0 = wp0;
  }

  double FrPropulsionForce::GetStraightRunWakeFraction() const {
    return m_wake_fraction0;
  }

  void FrPropulsionForce::SetK1(double k1) {
    m_K1 = k1;
  }

  double FrPropulsionForce::GetK1() const {
    return m_K1;
  }

  void FrPropulsionForce::SetThrustDeductionFactor(double tp) {
    m_thrust_deduction_factor = tp;
  }

  double FrPropulsionForce::GetThrustDeductionFactor() const {
    return m_thrust_deduction_factor;
  }

  void FrPropulsionForce::SetCorrectionFactor(double ku) {
    m_correction_factor = ku;
  }

  double FrPropulsionForce::GetCorrectionFactor() const {
    return m_correction_factor;
  }

  double FrPropulsionForce::GetLongitudinalVelocity() const {
    return m_longitudinal_velocity;
  }

  double FrPropulsionForce::GetTransversalVelocity() const {
    return m_transversal_velocity;
  }
}