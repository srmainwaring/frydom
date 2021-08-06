//
// Created by lletourn on 21/05/2021.
//

#include "FrPropellerForce.h"
#include "frydom/environment/FrEnvironmentInc.h"
#include "frydom/core/body/FrBody.h"

namespace frydom {

  FrPropellerForce::FrPropellerForce(const std::string &name,
                                     FrBody *body,
                                     Position propellerPositionInBody,
                                     FRAME_CONVENTION fc) :
      FrPropulsionActuator(name, "FrPropulsionName", body), m_name(name),
      m_positionInBody(propellerPositionInBody),
      m_thrust_deduction_factor(0.),
      m_K1(4),
      m_wake_fraction0(0.),
      m_correction_factor(1.),
      m_diameter(1.0),
      m_rotational_velocity(0.),
      m_screwDirection(RIGHT_HANDED) {

    if (IsNED(fc)) internal::SwapFrameConvention(m_positionInBody);

  }

  void FrPropellerForce::SetDiameter(double D) {
    m_diameter = D;
  }

  double FrPropellerForce::GetDiameter() const {
    return m_diameter;
  }


  double FrPropellerForce::ComputeLongitudinalVelocity() const {
    auto body = GetBody();
    auto environment = body->GetSystem()->GetEnvironment();
    auto propellerVelocityInWorld = body->GetVelocityInWorldAtPointInBody(GetPositionInBody(), NWU);
    const Velocity propellerRelativeVelocity = -environment->GetRelativeVelocityInFrame(FrFrame(),
                                                                                        propellerVelocityInWorld, WATER,
                                                                                        NWU);
    auto propellerRelativeVelocityInBody = body->ProjectVectorInBody(propellerRelativeVelocity, NWU);
    auto sidewashAngle = propellerRelativeVelocityInBody.GetProjectedAngleAroundZ(RAD);
    return m_correction_factor * propellerRelativeVelocityInBody.GetVx() * (1. - GetWakeFraction(sidewashAngle));
  }

  double FrPropellerForce::GetWakeFraction(double sidewashAngle) const {
    return m_wake_fraction0 * exp(-m_K1 * sidewashAngle * sidewashAngle);
  }

  void FrPropellerForce::SetStraightRunWakeFraction(double wp0) {
    m_wake_fraction0 = wp0;
  }

  double FrPropellerForce::GetStraightRunWakeFraction() const {
    return m_wake_fraction0;
  }

  void FrPropellerForce::SetK1(double k1) {
    m_K1 = k1;
  }

  double FrPropellerForce::GetK1() const {
    return m_K1;
  }

  void FrPropellerForce::SetThrustDeductionFactor(double tp) {
    m_thrust_deduction_factor = tp;
  }

  double FrPropellerForce::GetThrustDeductionFactor() const {
    return m_thrust_deduction_factor;
  }

  void FrPropellerForce::SetCorrectionFactor(double ku) {
    m_correction_factor = ku;
  }

  double FrPropellerForce::GetCorrectionFactor() const {
    return m_correction_factor;
  }

  void FrPropellerForce::Compute(double time) {

    auto generalizedForce = ComputeGeneralizedForceInBody();
    SetForceTorqueInBodyAtPointInBody(generalizedForce.GetForce(), generalizedForce.GetTorque(), m_positionInBody,
                                       NWU);

  }

  Position FrPropellerForce::GetPositionInBody() const {
    return m_positionInBody;
  }

  double FrPropellerForce::GetRotationalVelocity(FREQUENCY_UNIT unit) const {
    return convert_frequency(m_rotational_velocity, RADS, unit);
  }

  void FrPropellerForce::SetRotationalVelocity(double omega, mathutils::FREQUENCY_UNIT unit) {
    m_rotational_velocity = convert_frequency(omega, unit, RADS);
  }

  void FrPropellerForce::SetRPM(double rpm) {
    SetRotationalVelocity(rpm, RPM);
  }

  double FrPropellerForce::GetRPM() const {
    return GetRotationalVelocity(RPM);
  }

  void FrPropellerForce::SetScrewDirection(SCREW_DIRECTION dir) {
    m_screwDirection = (dir == RIGHT_HANDED ? 1 : -1);
//    m_screwDirection = dir;
  }

  SCREW_DIRECTION FrPropellerForce::GetScrewDirection() const {
    return m_screwDirection == 1 ? RIGHT_HANDED : LEFT_HANDED;
  }

  signed int FrPropellerForce::GetScrewDirectionSign() const {
    return m_screwDirection;
  }

//  signed int FrPropellerForce::GetScrewDirectionSign() const {
//    return m_screwDirection == RIGHT_HANDED ? 1 : -1;
////    signed int result = 0;
////    switch (m_screwDirection) {
////      case LEFT_HANDED: {
////        result = -1;
////        break;
////      }
////      case RIGHT_HANDED: {
////        result = 1;
////        break;
////      }
////    }
////    return result;
//  }

  double FrPropellerForce::GetPower() const {
    return GetRotationalVelocity(RADS) * c_torque;
  }

  double FrPropellerForce::GetThrust() const {
    return c_thrust;
  }

  double FrPropellerForce::GetTorque() const {
    return c_torque;
  }

  void FrPropellerForce::Initialize() {
    ReadCoefficientsFile();
  }

  void FrPropellerForce::DefineLogMessages() {
    auto msg = NewMessage("FrForce", "Force message");

    msg->AddField<double>("Time", "s", "Current time of the simulation",
                          [this]() { return m_chronoForce->GetChTime(); });

    msg->AddField<double>("Thrust", "N", "Thrust delivered by the propeller",
                          [this]() { return GetThrust(); });

    msg->AddField<double>("Torque", "Nm", "Torque delivered by the propeller",
                          [this]() { return GetTorque(); });

    msg->AddField<double>("Power", "W", "Power delivered by the propeller",
                          [this]() { return GetPower(); });

    msg->AddField<double>("uPA", "m/s", "Longitudinal velocity at the rudder position, in body reference frame",
                          [this]() { return ComputeLongitudinalVelocity(); });

    msg->AddField<double>("RotationalVelocity", "rad/s", "Rotational velocity",
                          [this]() { return GetRotationalVelocity(RADS); });

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
}