//
// Created by lletourn on 25/05/2021.
//

#include "FrRudderForce.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/logging/FrEventLogger.h"

namespace frydom {

//  FrFoilForce::FrFoilForce(const std::string &name, FrBody *body, const std::string &fileCoefficients) :
//      FrForce(name, "FrFoilForce", body),
//      m_projectedLateralArea(1.0), c_fluidDensity(1025.) {
//    ReadCoefficientsFile(fileCoefficients);
//  }
//
//  void FrFoilForce::SetProjectedLateralArea(double area) {
//    m_projectedLateralArea = area;
//  }
//
//  double FrFoilForce::GetProjectedLateralArea() const {
//    return m_projectedLateralArea;
//  }
//
//  void FrFoilForce::Compute(double time) {
//
//    auto foilRelativeVelocityInWorld = GetInflowVelocityInWorld();
//
//    auto foilGeneralizedForceInWorld = ComputeGeneralizedForceInWorld(foilRelativeVelocityInWorld);
//
//    SetForceTorqueInWorldAtPointInBody(foilGeneralizedForceInWorld.GetForce(),
//                                       foilGeneralizedForceInWorld.GetTorque(),
//                                       GetPositionInBody(), NWU);
//
//  }
//
//  mathutils::Vector3d<double> FrFoilForce::GetCoefficients(double attackAngle) const {
//    return m_coefficients.Eval("coefficients", attackAngle);
//  }
//

  FrRudderForce::FrRudderForce(const std::string &name, FrBody *body,
                               const std::string &fileCoefficients, const std::shared_ptr<FrNode> &node)
      : FrForce(name, "FrRudderForce", body), m_rudderAngle(0.), m_rudderNode(node), m_projectedLateralArea(1),
      m_wakeFraction0(0.4), m_K1(4), is_hullRudderInteraction(false),
      m_k(2.), m_beta1(1.3), m_beta2(MU_PI_2), m_K2(0.5), m_K3(0.45),
      c_fileCoefficients(fileCoefficients) {
  }
  void FrRudderForce::Initialize() {
    FrForce::Initialize();
    ReadCoefficientsFile(c_fileCoefficients);
//    c_fluidDensity = GetBody()->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
  }

  void FrRudderForce::SetProjectedLateralArea(double area) {
    m_projectedLateralArea = area;
  }

  double FrRudderForce::GetProjectedLateralArea() const {
    return m_projectedLateralArea;
  }

  Velocity FrRudderForce::GetInflowVelocityInWorld() const {

    auto body = GetBody();
    auto environment = body->GetSystem()->GetEnvironment();
    auto rudderVelocityInWorld = body->GetVelocityInWorldAtPointInBody(GetPositionInBody(), NWU);
    auto rudderRelativeVelocity = environment->GetRelativeVelocityInFrame(FrFrame(), rudderVelocityInWorld, WATER,
                                                                          NWU);

    if (is_hullRudderInteraction) {
      auto rudderRelativeVelocityInBody = body->ProjectVectorInBody(rudderRelativeVelocity, NWU);
      auto sidewashAngle = rudderRelativeVelocityInBody.GetProjectedAngleAroundZ(RAD);
      auto uRA = rudderRelativeVelocityInBody.GetVx() * (1. - GetWakeFraction(sidewashAngle));
      auto specialSidewashAngle = ComputeSpecialSidewashAngle();
      auto vRA = rudderRelativeVelocityInBody.GetVy() * Kappa(specialSidewashAngle);
      return body->ProjectVectorInWorld(Velocity(uRA, vRA, 0.0), NWU);
    } else
      return rudderRelativeVelocity;
  }

  double FrRudderForce::GetWakeFraction(double sidewashAngle) const {
    return m_wakeFraction0 * exp(-m_K1 * sidewashAngle * sidewashAngle);
  }

  void FrRudderForce::SetRudderAngle(double angle) {
    m_rudderAngle = angle;
  }

  double FrRudderForce::GetRudderAngle() const {
    return m_rudderAngle;
  }

  void FrRudderForce::SetStraightRunWakeFraction(double w0) {
    m_wakeFraction0 = w0;
  }

  double FrRudderForce::GetStraightRunWakeFraction() const {
    return m_wakeFraction0;
  }

  void FrRudderForce::SetTransverseVelocityCorrection(double k) {
    m_k = k;
  }

  Position FrRudderForce::GetPositionInBody() const {
    return m_rudderNode->GetNodePositionInBody(NWU);
  }

  FrFrame FrRudderForce::GetInflowFrame(Velocity inflowVelocity) const {

    auto x = inflowVelocity;
    x.normalize();
    auto z = m_rudderNode->GetFrameInWorld().GetZAxisInParent(NWU);
    auto y = z.cross(x);
    if (y.isZero(1E-3)) {
      event_logger::error("FrRudderForce", GetName(), "inflowVelocity is along Rudder axis");
      return FrFrame();
    }
    x = y.cross(z);

    FrFrame inflowFrame;
    inflowFrame.Set(Position(), x, y, z, NWU);

    return inflowFrame;
  }

  double FrRudderForce::GetAttackAngle(Velocity inflowVelocity) const {
    return GetRudderAngle() - GetDriftAngle(inflowVelocity);
  }

  double FrRudderForce::GetDriftAngle(Velocity inflowVelocity) const {
    return GetBody()->ProjectVectorInBody(inflowVelocity, NWU).GetProjectedAngleAroundZ(RAD);
  }

  void FrRudderForce::Compute(double time) {

    auto foilRelativeVelocityInWorld = GetInflowVelocityInWorld();

    auto foilGeneralizedForceInWorld = ComputeGeneralizedForceInWorld(foilRelativeVelocityInWorld);

    SetForceTorqueInWorldAtPointInBody(foilGeneralizedForceInWorld.GetForce(),
                                       foilGeneralizedForceInWorld.GetTorque(),
                                       GetPositionInBody(), NWU);

  }

  GeneralizedForce FrRudderForce::ComputeGeneralizedForceInWorld(Velocity inflowVelocity) const {
    GeneralizedForce rudderForce;
    rudderForce.SetNull();

    if (not inflowVelocity.isZero(1E-3)) {

      auto attackAngle = GetAttackAngle(inflowVelocity);

      // projection of the rudder velocity in the body COG reference frame
      auto rudderVelocityInBody = GetBody()->ProjectVectorInBody(inflowVelocity, NWU);
      // vertical component in the body reference frame should not be considered
      rudderVelocityInBody.GetVz() = 0.;
      auto squaredNormVelocity = rudderVelocityInBody.norm();
      squaredNormVelocity *= squaredNormVelocity;

      auto coefficients = GetCoefficients(attackAngle);

      auto density = GetBody()->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

      auto drag = 0.5 * density * coefficients[0] * m_projectedLateralArea * squaredNormVelocity;
      auto lift = 0.5 * density * coefficients[1] * m_projectedLateralArea * squaredNormVelocity;
      auto torque = 0.5 * density * coefficients[2] * m_projectedLateralArea * squaredNormVelocity;

      auto inflowFrame = GetInflowFrame(inflowVelocity);

      rudderForce.SetForce(inflowFrame.ProjectVectorFrameInParent(Force(drag, lift, 0.), NWU));
      rudderForce.SetTorque(inflowFrame.ProjectVectorFrameInParent(Torque(0., 0., torque), NWU));

    }
    return rudderForce;
  }

  double FrRudderForce::ComputeSpecialSidewashAngle() const {
    auto vesselVelocity = GetBody()->GetVelocityInHeadingFrame(NWU);

    auto r = GetBody()->GetAngularVelocityInWorld(NWU).GetWz();
    auto x_r = GetPositionInBody().GetX();

    vesselVelocity.GetVy() += + m_k * r * x_r;

    return vesselVelocity.GetProjectedAngleAroundZ(RAD);
  }

  double FrRudderForce::Kappa(double specialSidewashAngle) const {
    auto beta = std::abs(specialSidewashAngle);
    double kappa;
    if (beta < m_beta1) {
      kappa = std::min(m_K2, m_K3 * beta);
    } else if (beta <= m_beta2 and beta >= m_beta1) {
      auto bv = 0.5/(m_beta2 - m_beta1);
      auto av = 0.5 - bv*m_beta1;
      kappa = av + bv*beta;
    }else {
      kappa = 1.0;
    }
    return kappa;
  }

  void FrRudderForce::ReadCoefficientsFile(const std::string &filename) {

  }

  mathutils::Vector3d<double> FrRudderForce::GetCoefficients(double attackAngle) const {
    return m_coefficients.Eval("coefficients", attackAngle);
  }

} // end namespace frydom