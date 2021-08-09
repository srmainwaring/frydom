//
// Created by lletourn on 08/06/2021.
//

#include "FrPropellerRudder.h"
#include "frydom/propulsion/FrPropulsionInc.h"

namespace frydom {

  FrPropellerRudder::FrPropellerRudder(const std::string &name, frydom::FrBody *body) :
      FrActuatorForceBase(name, "FrPropellerRudder", body), has_interactions(true),
      c_propellerForceInBody(Force(), Torque(), Position(), NWU),
      c_rudderForceInWorld(Force(), Torque(), Position(), NWU) {

  }

  FrFirstQuadrantPropellerForce *
  FrPropellerRudder::Add_FirstQuadrantPropeller(const std::string &name,
                                                Position propellerPositionInBody,
                                                const std::string &filename,
                                                FRAME_CONVENTION fc) {

    m_propellerForce = std::make_shared<FrFirstQuadrantPropellerForce>(name,
                                                                       GetBody(),
                                                                       propellerPositionInBody,
                                                                       filename,
                                                                       fc);
    return dynamic_cast<FrFirstQuadrantPropellerForce *>(m_propellerForce.get());

  }

  FrFourQuadrantPropellerForce *
  FrPropellerRudder::Add_FourQuadrantPropeller(const std::string &name,
                                               Position propellerPositionInBody,
                                               const std::string &filename,
                                               FRAME_CONVENTION fc) {

    m_propellerForce = std::make_shared<FrFourQuadrantPropellerForce>(name,
                                                                      GetBody(),
                                                                      propellerPositionInBody,
                                                                      filename,
                                                                      fc);
    return dynamic_cast<FrFourQuadrantPropellerForce *>(m_propellerForce.get());

  }

  FrCPPForce *
  FrPropellerRudder::Add_ControllablePitchPropeller(const std::string &name,
                                                    Position propellerPositionInBody,
                                                    const std::string &filename,
                                                    FRAME_CONVENTION fc) {

    m_propellerForce = std::make_shared<FrCPPForce>(name,
                                                    GetBody(),
                                                    propellerPositionInBody,
                                                    filename,
                                                    fc);
    return dynamic_cast<FrCPPForce *>(m_propellerForce.get());

  }

  FrRudderForce *FrPropellerRudder::Add_Rudder(const std::string &name,
                                               const std::shared_ptr<FrNode> &node,
                                               const std::string &filename) {

    m_rudderForce = std::make_shared<FrRudderForce>(name, GetBody(), node, filename);
    return m_rudderForce.get();

  }

  FrFlapRudderForce *FrPropellerRudder::Add_FlapRudder(const std::string &name,
                                                       const std::shared_ptr<FrNode> &node,
                                                       const std::string &filename) {

    m_rudderForce = std::make_shared<FrFlapRudderForce>(name, GetBody(), node, filename);
    return dynamic_cast<FrFlapRudderForce *>(m_rudderForce.get());

  }

  void FrPropellerRudder::Compute(double time) {

    // Propeller force
    auto propellerForce = m_propellerForce->ComputeGeneralizedForceInBody();
    c_propellerForceInBody = {propellerForce, m_propellerForce->GetPositionInBody(), NWU};
    SetForceTorqueInBodyAtPointInBody(propellerForce.GetForce(), propellerForce.GetTorque(),
                                      m_propellerForce->GetPositionInBody(), NWU);
    c_uPA = m_propellerForce->ComputeLongitudinalVelocity();

    // Rudder force
    const auto rudderRelativeVelocityInWorld = m_rudderForce->GetRudderRelativeVelocityInWorld();
    auto rudderForceInWorld = m_rudderForce->ComputeGeneralizedForceInWorld(rudderRelativeVelocityInWorld);

    auto rudderRelativeVelocityInBody = GetBody()->ProjectVectorInBody(rudderRelativeVelocityInWorld, NWU);
    c_uRA = rudderRelativeVelocityInBody.GetVx();
    c_vRA = rudderRelativeVelocityInBody.GetVy();
    // Propeller - Rudder interactions
    if (c_uPA != 0. and has_interactions) {
      auto body = GetBody();
      auto environment = GetSystem()->GetEnvironment();
      auto rudderVelocityInWorld = body->GetVelocityInWorldAtPointInBody(m_rudderForce->GetPositionInBody(), NWU);
      auto u_0 = -environment->GetRelativeVelocityInFrame(GetBody()->GetHeadingFrame(), rudderVelocityInWorld, WATER, NWU).GetVx();

      c_vRP = c_vRA;
      ComputeVelocityInSlipStream(u_0, c_uRA, c_uPA, m_propellerForce->GetThrust(), c_uRP, c_A_RP, c_kd);
      double area_ratio = c_A_RP / m_rudderForce->GetProjectedLateralArea();

      // Inside slipstream rudder force
      auto propellerVelocityInSlipstream = GetBody()->ProjectVectorInWorld(Velocity(c_uRP, c_vRP, 0.), NWU);
      auto insideSlipstreamRudderForce = m_rudderForce->ComputeGeneralizedForceInWorld(propellerVelocityInSlipstream);

      rudderForceInWorld *= (1. - area_ratio);
      rudderForceInWorld += area_ratio * c_kd * insideSlipstreamRudderForce;
    }

    c_rudderForceInWorld = {rudderForceInWorld, m_rudderForce->GetPositionInBody(), NWU};

    Force totalForce = GetForceInWorld(NWU) + rudderForceInWorld.GetForce();
    Torque totalTorqueAtRudder = GetTorqueInWorldAtPointInBody(m_rudderForce->GetPositionInBody(), NWU) +
                                 rudderForceInWorld.GetTorque();

    SetForceTorqueInWorldAtPointInBody(totalForce, totalTorqueAtRudder, m_rudderForce->GetPositionInBody(), NWU);

  }


  void
  FrPropellerRudder::ComputeVelocityInSlipStream(double u_0, double u_ra, double u_pa, double thrust, double &u_rp,
                                                 double &A_rp, double &kd) {

    auto rho = GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
    auto Dp = m_propellerForce->GetDiameter();
    auto A0 = 0.25 * MU_PI * Dp * Dp;
    auto xrp = m_longitudinalDistancePropellerRudder;
    auto br = m_rudderForce->GetRootChord();
    auto propellerLoading = 2 * std::abs(thrust) / (rho * u_pa * u_pa * A0);

    auto w_ainf = u_pa * (std::sqrt(1 + propellerLoading) - 1);
    auto Kappa = [](double T, double xrp) { if (xrp * T >= 0) return 1.; else return 0.7; };
    auto w_a = 0.34 * (1 + xrp / std::sqrt(1 + xrp * xrp) * Kappa(thrust, xrp)) * mathutils::sgn(thrust) * w_ainf;
    u_rp = u_pa + w_a;

    auto r_rp = 0.5 * Dp * std::sqrt(std::abs(0.5 * u_0 / u_rp));

    kd = std::pow(std::abs(u_ra / u_rp), 2. * std::pow(2. / (2. + std::sqrt(MU_PI) * r_rp / (2 * br)), 8));

    A_rp = 2. * r_rp / m_rudderForce->GetHeight() * m_rudderForce->GetProjectedLateralArea();

  }

  void FrPropellerRudder::Initialize() {
    FrForce::Initialize();
    m_propellerForce->Initialize();
    m_rudderForce->Initialize();
    m_rudderForce->ActivateHullRudderInteraction(IsInteractions());

    m_longitudinalDistancePropellerRudder =
        m_propellerForce->GetPositionInBody().GetX() - m_rudderForce->GetPositionInBody().GetX();
  }

  void FrPropellerRudder::SetRudderAngle(double angle, ANGLE_UNIT unit) {
    m_rudderForce->SetRudderAngle(angle, unit);
  }

  void FrPropellerRudder::SetPropellerRotationalVelocity(double omega, mathutils::FREQUENCY_UNIT unit) {
    m_propellerForce->SetRotationalVelocity(omega * m_propellerForce->GetScrewDirectionSign(), unit);

  }

  void FrPropellerRudder::ActivateInteractions(bool val) {
    has_interactions = val;
  }

  bool FrPropellerRudder::IsInteractions() const {
    return has_interactions;
  }

  void FrPropellerRudder::DefineLogMessages() {

    // Propeller logs
    auto prop_msg = NewMessage("_propeller", "propeller force message");

    prop_msg->AddField<double>("Time", "s", "Current time of the simulation",
                               [this]() { return m_chronoForce->GetChTime(); });

    prop_msg->AddField<double>("Thrust", "N", "Thrust delivered by the propeller",
                               [this]() { return m_propellerForce->GetThrust(); });

    prop_msg->AddField<double>("Torque", "Nm", "Torque delivered by the propeller",
                               [this]() { return m_propellerForce->GetTorque(); });

    prop_msg->AddField<double>("Power", "W", "Power delivered by the propeller",
                               [this]() { return m_propellerForce->GetPower(); });

    prop_msg->AddField<double>("uPA", "m/s", "Longitudinal velocity at the rudder position, in body reference frame",
                               [this]() { return c_uPA; });

    prop_msg->AddField<double>("RotationalVelocity", "rad/s", "Rotational velocity",
                               [this]() { return m_propellerForce->GetRotationalVelocity(RADS); });

    prop_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("ForceInBody", "N", fmt::format("force in body reference frame in {}", GetLogFC()),
         [this]() { return GetPropellerForceInBody(GetLogFC()); });

    prop_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TorqueInBodyAtCOG", "Nm", fmt::format("torque in body reference frame, at COG, in {}", GetLogFC()),
         [this]() { return GetPropellerTorqueInBodyAtCOG(GetLogFC()); });

//    prop_msg->AddField<Eigen::Matrix<double, 3, 1>>
//        ("TorqueInBodyAtProp", "Nm", fmt::format("torque in body reference frame, at prop, in {}", GetLogFC()),
//         [this]() { return GetPropellerTorqueInBodyAtPropeller(GetLogFC()); });

    prop_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("ForceInWorld", "N", fmt::format("force in world reference frame in {}", GetLogFC()),
         [this]() { return GetPropellerForceInWorld(GetLogFC()); });

    prop_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TorqueInWorldAtCOG", "Nm", fmt::format("torque in world reference frame, at COG, in {}", GetLogFC()),
         [this]() { return GetPropellerTorqueInWorldAtCOG(GetLogFC()); });

//    prop_msg->AddField<Eigen::Matrix<double, 3, 1>>
//        ("TorqueInWorldAtProp", "Nm", fmt::format("torque in world reference frame, at prop, in {}", GetLogFC()),
//         [this]() { return GetPropellerTorqueInWorldAtPropeller(GetLogFC()); });


    // Rudder logs
    auto rudder_msg = NewMessage("_rudder", "rudder force message");

    rudder_msg->AddField<double>("Time", "s", "Current time of the simulation",
                                 [this]() { return m_chronoForce->GetChTime(); });

    rudder_msg->AddField<double>("RudderAngle", "rad", "Rudder angle",
                                 [this]() { return m_rudderForce->GetRudderAngle(RAD); });

    rudder_msg->AddField<double>("DriftAngle_RA", "rad", "Drift angle",
                                 [this]() {
                                   return m_rudderForce->GetDriftAngle(
                                       GetBody()->ProjectVectorInWorld(Velocity(c_uRA, c_vRA, 0.), NWU));
                                 });

    rudder_msg->AddField<double>("DriftAngle_RP", "rad", "Drift angle",
                                 [this]() {
                                   return m_rudderForce->GetDriftAngle(
                                       GetBody()->ProjectVectorInWorld(Velocity(c_uRP, c_vRP, 0.), NWU));
                                 });

    rudder_msg->AddField<double>("AttackAngle", "rad", "Attack angle",
                                 [this]() {
                                   return m_rudderForce->GetAttackAngle(
                                       m_rudderForce->GetRudderRelativeVelocityInWorld());
                                 });

    rudder_msg->AddField<double>("uRA", "m/s", "Longitudinal velocity",
                                 [this]() { return c_uRA; });

    rudder_msg->AddField<double>("vRA", "m/s", "transversal velocity",
                                 [this]() { return c_vRA; });

    rudder_msg->AddField<double>("uRP", "m/s", "Longitudinal velocity",
                                 [this]() { return c_uRP; });

    rudder_msg->AddField<double>("vRP", "m/s", "transversal velocity",
                                 [this]() { return c_vRP; });

    rudder_msg->AddField<double>("kd", "", "",
                                 [this]() { return c_kd; });

    rudder_msg->AddField<double>("slipstream_area_ratio", "", "Area ratio",
                                 [this]() { return c_A_RP/m_rudderForce->GetProjectedLateralArea(); });

    rudder_msg->AddField<double>("Drag", "N", "Drag delivered by the rudder",
                                 [this]() { return m_rudderForce->GetDrag(); });

    rudder_msg->AddField<double>("Lift", "N", "Lift delivered by the rudder",
                                 [this]() { return m_rudderForce->GetLift(); });

    rudder_msg->AddField<double>("Torque", "Nm", "Torque delivered by the rudder",
                                 [this]() { return m_rudderForce->GetTorque(); });

    rudder_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("InflowVelocityInWorld", "m/s", fmt::format("Inflow velocity in World reference frame in {}", GetLogFC()),
         [this]() { return m_rudderForce->GetRudderRelativeVelocityInWorld(); });

    rudder_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("ForceInBody", "N", fmt::format("force in body reference frame in {}", GetLogFC()),
         [this]() { return GetRudderForceInBody(GetLogFC()); });

    rudder_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TorqueInBodyAtCOG", "Nm", fmt::format("torque in body reference frame, at COG, in {}", GetLogFC()),
         [this]() { return GetRudderTorqueInBodyAtCOG(GetLogFC()); });

//    rudder_msg->AddField<Eigen::Matrix<double, 3, 1>>
//        ("TorqueInBodyAtRudder", "Nm", fmt::format("torque in body reference frame, at rudder, in {}", GetLogFC()),
//         [this]() { return GetRudderTorqueInBodyAtRudder(GetLogFC()); });

    rudder_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("ForceInWorld", "N", fmt::format("force in world reference frame in {}", GetLogFC()),
         [this]() { return GetRudderForceInWorld(GetLogFC()); });

    rudder_msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TorqueInWorldAtCOG", "Nm", fmt::format("torque in world reference frame, at COG, in {}", GetLogFC()),
         [this]() { return GetRudderTorqueInWorldAtCOG(GetLogFC()); });

//    rudder_msg->AddField<Eigen::Matrix<double, 3, 1>>
//        ("TorqueInWorldAtRudder", "Nm", fmt::format("torque in world reference frame, at rudder, in {}", GetLogFC()),
//         [this]() { return GetRudderTorqueInWorldAtRudder(GetLogFC()); });


  }

  //Propeller

  Force FrPropellerRudder::GetPropellerForceInWorld(FRAME_CONVENTION fc) const {
    return GetBody()->ProjectVectorInWorld(GetPropellerForceInBody(fc), fc);
  }

  Force FrPropellerRudder::GetPropellerForceInBody(FRAME_CONVENTION fc) const {
    Force force = c_propellerForceInBody.GetForce();
    if (IsNED(fc)) internal::SwapFrameConvention(force);
    return force;
  }

  Torque FrPropellerRudder::GetPropellerTorqueInWorldAtPropeller(FRAME_CONVENTION fc) const {
    return GetBody()->ProjectVectorInWorld(GetPropellerTorqueInBodyAtPropeller(fc), fc);
  }

  Torque FrPropellerRudder::GetPropellerTorqueInBodyAtPropeller(FRAME_CONVENTION fc) const {
    return c_propellerForceInBody.GetTorqueAtPoint(m_propellerForce->GetPositionInBody(), fc);
  }

  Torque FrPropellerRudder::GetPropellerTorqueInWorldAtCOG(FRAME_CONVENTION fc) const {
    return GetBody()->ProjectVectorInWorld(GetPropellerTorqueInBodyAtCOG(fc), fc);
  }

  Torque FrPropellerRudder::GetPropellerTorqueInBodyAtCOG(FRAME_CONVENTION fc) const {
    return c_propellerForceInBody.GetTorqueAtPoint(GetBody()->GetCOG(fc), fc);
  }

  // Rudder

  Force FrPropellerRudder::GetRudderForceInWorld(FRAME_CONVENTION fc) const {
    Force force = c_rudderForceInWorld.GetForce();
    if (IsNED(fc)) internal::SwapFrameConvention(force);
    return force;
  }

  Force FrPropellerRudder::GetRudderForceInBody(FRAME_CONVENTION fc) const {
    return GetBody()->ProjectVectorInBody(GetRudderForceInWorld(fc), fc);
  }

  Torque FrPropellerRudder::GetRudderTorqueInWorldAtRudder(FRAME_CONVENTION fc) const {
    auto rudderPositionInWorld = GetBody()->GetPointPositionInWorld(m_rudderForce->GetPositionInBody(), fc);
    return c_rudderForceInWorld.GetTorqueAtPoint(rudderPositionInWorld, fc);
  }

  Torque FrPropellerRudder::GetRudderTorqueInBodyAtRudder(FRAME_CONVENTION fc) const {
    return GetBody()->ProjectVectorInBody(GetRudderTorqueInWorldAtRudder(fc), fc);
  }

  Torque FrPropellerRudder::GetRudderTorqueInWorldAtCOG(FRAME_CONVENTION fc) const {
    return c_propellerForceInBody.GetTorqueAtPoint(GetBody()->GetCOGPositionInWorld(fc), fc);
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