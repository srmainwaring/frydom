//
// Created by lletourn on 08/06/2021.
//

#include "FrPropellerRudder.h"
#include "frydom/propulsion/FrPropulsionInc.h"

namespace frydom {

  FrPropellerRudder::FrPropellerRudder(const std::string &name, frydom::FrBody *body) :
      FrForce(name, "FrPropellerRudder", body),
      c_propellerForce(Force(), Torque(), Position(), NWU),
      c_rudderForce(Force(), Torque(), Position(), NWU){

  }

  FrFirstQuadrantPropellerForce *
  FrPropellerRudder::Add_FirstQuadrantPropeller(const std::string &name, Position propellerPositionInBody,
                                                const std::string &filename,
                                                FRAME_CONVENTION fc) {
    m_propellerForce = std::make_shared<FrFirstQuadrantPropellerForce>(name, GetBody(), propellerPositionInBody,
                                                                       filename, fc);
    return dynamic_cast<FrFirstQuadrantPropellerForce *>(m_propellerForce.get());
  }

  FrFourQuadrantPropellerForce *
  FrPropellerRudder::Add_FourQuadrantPropeller(const std::string &name, Position propellerPositionInBody,
                                               const std::string &filename,
                                               FRAME_CONVENTION fc) {
    m_propellerForce = std::make_shared<FrFourQuadrantPropellerForce>(name, GetBody(), propellerPositionInBody,
                                                                      filename, fc);
    return dynamic_cast<FrFourQuadrantPropellerForce *>(m_propellerForce.get());
  }

  FrCPPForce *
  FrPropellerRudder::Add_ControllablePitchPropeller(const std::string &name, Position propellerPositionInBody,
                                                    const std::string &filename,
                                                    FRAME_CONVENTION fc) {
    m_propellerForce = std::make_shared<FrCPPForce>(name, GetBody(), propellerPositionInBody,
                                                    filename, fc);
    return dynamic_cast<FrCPPForce *>(m_propellerForce.get());
  }

  FrRudderForce *FrPropellerRudder::Add_Rudder(const std::string &name, const std::shared_ptr<FrNode> &node,
                                               const std::string &filename) {
    m_rudderForce = std::make_shared<FrRudderForce>(name, GetBody(), node, filename);
    return m_rudderForce.get();
  }

  FrFlapRudderForce *FrPropellerRudder::Add_FlapRudder(const std::string &name, const std::shared_ptr<FrNode> &node,
                                                       const std::string &filename) {
    m_rudderForce = std::make_shared<FrFlapRudderForce>(name, GetBody(), node, filename);
    return dynamic_cast<FrFlapRudderForce *>(m_rudderForce.get());
  }

  void FrPropellerRudder::Compute(double time) {

//    auto Kappa = [](double T, double xrp) { if (xrp * T >= 0) return 1.; else return 0.7; };
//
//    auto rho = GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
//    auto Dp = m_propellerForce->GetDiameter();
//    auto A0 = 0.25 * MU_PI * Dp * Dp;
//    auto xrp = m_longitudinalDistancePropellerRudder;
//    auto A_R = m_rudderForce->GetProjectedLateralArea();
//    auto br = m_rudderForce->GetRootChord();
//
//    auto vesselVelocityInBody = GetBody()->GetVelocityInBody(NWU);
//    auto vesselApparentVelocity = GetSystem()->GetEnvironment()->GetRelativeVelocityInFrame(GetBody()->GetFrameAtCOG(),
//                                                                                            vesselVelocityInBody, WATER,
//                                                                                            NWU);
//    auto u0 = vesselApparentVelocity.GetVx();

    // Propeller force
    c_propellerForce = {m_propellerForce->ComputeGeneralizedForceInWorld(), m_propellerForce->GetPositionInBody(), NWU};
    SetForceTorqueInWorldAtPointInBody(c_propellerForce.GetForce(),
                                       c_propellerForce.GetTorqueAtPoint(m_propellerForce->GetPositionInBody(), NWU),
                                       m_propellerForce->GetPositionInBody(), NWU);
//    auto T = GetBody()->ProjectVectorInBody(c_propellerForce.GetForce(), NWU).GetFx();
//    auto u_PA = m_propellerForce->ComputeLongitudinalVelocity();

    // Rudder force
    const auto inflowRelativeVelocityInWorld = m_rudderForce->GetInflowVelocityInWorld();
//    auto u_RA = -GetBody()->ProjectVectorInBody(inflowRelativeVelocityInWorld, NWU).GetVx();

    c_rudderForce = {m_rudderForce->ComputeGeneralizedForceInWorld(inflowRelativeVelocityInWorld),
                     m_rudderForce->GetPositionInBody(), NWU};

    Force totalForce = c_propellerForce.GetForce() + c_rudderForce.GetForce();
    Torque temp = GetTorqueInWorldAtPointInBody(m_rudderForce->GetPositionInBody(), NWU);
    Torque totalTorqueAtRudder = temp + //c_propellerForce.GetTorqueAtPoint(m_rudderForce->GetPositionInBody(), NWU) +
                                 c_rudderForce.GetTorqueAtPoint(m_rudderForce->GetPositionInBody(), NWU);

    // Propeller - Rudder interactions
//    if (u_PA != 0. and false) {
//      auto propellerLoading = 2 * std::abs(T) / (rho * u_PA * u_PA * A0);
//
//      auto w_ainf = u_PA * (std::sqrt(1 + propellerLoading) - 1);
//      auto w_a = 0.34 * (1 + xrp / std::sqrt(1 + xrp * xrp) * Kappa(T, xrp)) * mathutils::sgn(T) * w_ainf;
//      auto u_RP = u_PA + w_a;
//      auto v_RP = -GetBody()->ProjectVectorInBody(inflowRelativeVelocityInWorld, NWU).GetVy();
//
//      auto r_RP = 0.5 * Dp * std::sqrt(std::abs(u0 / u_RP));
//
//      auto kd = std::pow(std::abs(u_RA / u_RP), 2. * std::pow(2. / (2. + std::sqrt(MU_PI) * r_RP / (2 * br)), 8));
//
//      auto A_RP = 2. * r_RP / m_rudderForce->GetHeight() * A_R;
//
//      // Inside slipstream rudder force
//      auto inflowPropellerSlipstream = GetBody()->ProjectVectorInBody(Velocity(-u_RP, -v_RP, 0.), NWU);
//      auto insideSlipstreamRudderForce = m_rudderForce->ComputeGeneralizedForceInWorld(inflowPropellerSlipstream);
//
//      totalTorqueAtRudder += A_RP / A_R * (kd * insideSlipstreamRudderForce.GetTorque()
//                                           - c_rudderForce.GetTorqueAtPoint(m_rudderForce->GetPositionInBody(), NWU));
//      totalForce += A_RP / A_R * (kd * insideSlipstreamRudderForce.GetForce()
//                                  - c_rudderForce.GetForce());
//    }

    SetForceTorqueInWorldAtPointInBody(totalForce, totalTorqueAtRudder, m_rudderForce->GetPositionInBody(), NWU);

  }

  void FrPropellerRudder::Initialize() {
    FrForce::Initialize();
    m_propellerForce->Initialize();
    m_rudderForce->Initialize();
    m_rudderForce->ActivateHullRudderInteraction(false);

    m_longitudinalDistancePropellerRudder =
        m_propellerForce->GetPositionInBody().GetX() - m_rudderForce->GetPositionInBody().GetX();
  }

  void FrPropellerRudder::SetRudderAngle(double angle, ANGLE_UNIT unit) {
    m_rudderForce->SetRudderAngle(angle, unit);
  }

  void FrPropellerRudder::SetPropellerRotationalVelocity(double omega, mathutils::FREQUENCY_UNIT unit) {
    m_propellerForce->SetRotationalVelocity(omega * m_propellerForce->GetScrewDirectionSign(), unit);

  }

  void FrPropellerRudder::DefineLogMessages() {

    // Propeller logs
    auto prop_msg = NewMessage("_propeller", "propeller force message");

    prop_msg->AddField<double>("Time", "s", "Current time of the simulation",
                               [this]() { return m_chronoForce->GetChTime(); });

    prop_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("ForceInBody", "N", fmt::format("force in body reference frame in {}", GetLogFC()),
         [this]() { return GetPropellerForceInBody(GetLogFC()); });

    prop_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TorqueInBodyAtCOG", "Nm", fmt::format("torque in body reference frame, at COG, in {}", GetLogFC()),
         [this]() { return GetPropellerTorqueInBodyAtCOG(GetLogFC()); });

    prop_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TorqueInBodyAtProp", "Nm", fmt::format("torque in body reference frame, at prop, in {}", GetLogFC()),
         [this]() { return GetPropellerTorqueInBodyAtPropeller(GetLogFC()); });

    prop_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("ForceInWorld", "N", fmt::format("force in world reference frame in {}", GetLogFC()),
         [this]() { return GetPropellerForceInWorld(GetLogFC()); });

    prop_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TorqueInWorldAtCOG", "Nm", fmt::format("torque in world reference frame, at COG, in {}", GetLogFC()),
         [this]() { return GetPropellerTorqueInWorldAtCOG(GetLogFC()); });

    prop_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TorqueInWorldAtProp", "Nm", fmt::format("torque in world reference frame, at prop, in {}", GetLogFC()),
         [this]() { return GetPropellerTorqueInWorldAtPropeller(GetLogFC()); });


    // Rudder logs
    auto rudder_msg = NewMessage("_rudder", "rudder force message");

    rudder_msg->AddField<double>("Time", "s", "Current time of the simulation",
                               [this]() { return m_chronoForce->GetChTime(); });

    rudder_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("ForceInBody", "N", fmt::format("force in body reference frame in {}", GetLogFC()),
         [this]() { return GetRudderForceInBody(GetLogFC()); });

    rudder_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TorqueInBodyAtCOG", "Nm", fmt::format("torque in body reference frame, at COG, in {}", GetLogFC()),
         [this]() { return GetRudderTorqueInBodyAtCOG(GetLogFC()); });

    rudder_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TorqueInBodyAtProp", "Nm", fmt::format("torque in body reference frame, at prop, in {}", GetLogFC()),
         [this]() { return GetRudderTorqueInBodyAtRudder(GetLogFC()); });

    rudder_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("ForceInWorld", "N", fmt::format("force in world reference frame in {}", GetLogFC()),
         [this]() { return GetRudderForceInWorld(GetLogFC()); });

    rudder_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TorqueInWorldAtCOG", "Nm", fmt::format("torque in world reference frame, at COG, in {}", GetLogFC()),
         [this]() { return GetRudderTorqueInWorldAtCOG(GetLogFC()); });

    rudder_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TorqueInWorldAtProp", "Nm", fmt::format("torque in world reference frame, at prop, in {}", GetLogFC()),
         [this]() { return GetRudderTorqueInWorldAtRudder(GetLogFC()); });




  }

  //Propeller

  Force FrPropellerRudder::GetPropellerForceInWorld(FRAME_CONVENTION fc) const {
    Force force = c_propellerForce.GetForce();
    if (IsNED(fc)) internal::SwapFrameConvention(force);
    return force;
  }

  Force FrPropellerRudder::GetPropellerForceInBody(FRAME_CONVENTION fc) const {
    return GetBody()->ProjectVectorInBody(GetPropellerForceInWorld(fc), fc);
  }

  Torque FrPropellerRudder::GetPropellerTorqueInWorldAtPropeller(FRAME_CONVENTION fc) const {
    return c_propellerForce.GetTorqueAtPoint(m_propellerForce->GetPositionInBody(), fc);
  }

  Torque FrPropellerRudder::GetPropellerTorqueInBodyAtPropeller(FRAME_CONVENTION fc) const {
    return GetBody()->ProjectVectorInBody(GetPropellerTorqueInWorldAtPropeller(fc), fc);
  }

  Torque FrPropellerRudder::GetPropellerTorqueInWorldAtCOG(FRAME_CONVENTION fc) const {
    return c_propellerForce.GetTorqueAtPoint(GetBody()->GetCOG(fc), fc);
  }

  Torque FrPropellerRudder::GetPropellerTorqueInBodyAtCOG(FRAME_CONVENTION fc) const {
    return GetBody()->ProjectVectorInBody(GetPropellerTorqueInWorldAtCOG(fc), fc);
  }

  // Rudder

  Force FrPropellerRudder::GetRudderForceInWorld(FRAME_CONVENTION fc) const {
    Force force = c_rudderForce.GetForce();
    if (IsNED(fc)) internal::SwapFrameConvention(force);
    return force;
  }

  Force FrPropellerRudder::GetRudderForceInBody(FRAME_CONVENTION fc) const {
    return GetBody()->ProjectVectorInBody(GetRudderForceInWorld(fc), fc);
  }

  Torque FrPropellerRudder::GetRudderTorqueInWorldAtRudder(FRAME_CONVENTION fc) const {
    return c_rudderForce.GetTorqueAtPoint(m_rudderForce->GetPositionInBody(), fc);
  }

  Torque FrPropellerRudder::GetRudderTorqueInBodyAtRudder(FRAME_CONVENTION fc) const {
    return GetBody()->ProjectVectorInBody(GetRudderTorqueInWorldAtRudder(fc), fc);
  }

  Torque FrPropellerRudder::GetRudderTorqueInWorldAtCOG(FRAME_CONVENTION fc) const {
    return c_propellerForce.GetTorqueAtPoint(GetBody()->GetCOG(fc), fc);
  }

  Torque FrPropellerRudder::GetRudderTorqueInBodyAtCOG(FRAME_CONVENTION fc) const {
    return GetBody()->ProjectVectorInBody(GetRudderTorqueInWorldAtCOG(fc), fc);
  }

  std::shared_ptr<FrPropellerRudder>
  make_propeller_rudder(const std::string &name, const std::shared_ptr<FrBody> &body) {
    auto force = std::make_shared<FrPropellerRudder>(name, body.get());
    body->AddExternalForce(force);
    return force;
  }

} // end namespace frydom