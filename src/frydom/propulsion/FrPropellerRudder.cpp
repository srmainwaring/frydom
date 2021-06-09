//
// Created by lletourn on 08/06/2021.
//

#include "FrPropellerRudder.h"
#include "frydom/propulsion/FrPropulsionInc.h"

namespace frydom {

  FrPropellerRudder::FrPropellerRudder(const std::string &name, frydom::FrBody *body) :
      FrForce(name, "FrPropellerRudder", body) {

  }

  FrFirstQuadrantPropellerForce *
  FrPropellerRudder::Add_FirstQuadrantPropeller(const std::string &name, Position propellerPositionInBody,
                                                const std::string &filename) {
    m_propellerForce = std::make_shared<FrFirstQuadrantPropellerForce>(name, GetBody(), propellerPositionInBody,
                                                                       filename);
    return dynamic_cast<FrFirstQuadrantPropellerForce *>(m_propellerForce.get());
  }

  FrFourQuadrantPropellerForce *
  FrPropellerRudder::Add_FourQuadrantPropeller(const std::string &name, Position propellerPositionInBody,
                                               const std::string &filename) {
    m_propellerForce = std::make_shared<FrFourQuadrantPropellerForce>(name, GetBody(), propellerPositionInBody,
                                                                      filename);
    return dynamic_cast<FrFourQuadrantPropellerForce *>(m_propellerForce.get());
  }

  FrCPPForce *
  FrPropellerRudder::Add_ControllablePitchPropeller(const std::string &name, Position propellerPositionInBody,
                                                    const std::string &filename) {
    m_propellerForce = std::make_shared<FrCPPForce>(name, GetBody(), propellerPositionInBody,
                                                    filename);
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

    auto Kappa = [](double T, double xrp) { if (xrp * T >= 0) return 1.; else return 0.7; };

    auto rho = GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
    auto Dp = m_propellerForce->GetDiameter();
    auto A0 = 0.25 * MU_PI * Dp * Dp;
    auto xrp = m_longitudinalDistancePropellerRudder;
    auto A_R = m_rudderForce->GetProjectedLateralArea();
    auto br = m_rudderForce->GetRootChord();

    auto vesselVelocityInBody = GetBody()->GetVelocityInBody(NWU);
    auto vesselApparentVelocity = GetSystem()->GetEnvironment()->GetRelativeVelocityInFrame(GetBody()->GetFrameAtCOG(),
                                                                                            vesselVelocityInBody, WATER,
                                                                                            NWU);
    auto u0 = vesselApparentVelocity.GetVx();

    auto inflowRelativeVelocityInWorld = m_rudderForce->GetInflowVelocityInWorld();
    auto u_RA = -GetBody()->ProjectVectorInBody(inflowRelativeVelocityInWorld, NWU).GetVx();

    // Propeller force
    auto propellerForce = m_propellerForce->ComputeGeneralizedForceInWorld();
    SetForceTorqueInWorldAtPointInBody(propellerForce.GetForce(), propellerForce.GetTorque(),
                                       m_propellerForce->GetPositionInBody(), NWU);
    auto T = GetBody()->ProjectVectorInBody(propellerForce.GetForce(), NWU).GetFx();
    auto u_PA = m_propellerForce->GetLongitudinalVelocity();
    auto propellerLoading = 2 * std::abs(T) / (rho * u_PA * u_PA * A0);

    // Propeller/rudder interactions
    auto w_ainf = u_PA * (std::sqrt(1 + propellerLoading) - 1);
    auto w_a = 0.34 * (1 + xrp / std::sqrt(1 + xrp * xrp) * Kappa(T, xrp)) * mathutils::sgn(T) * w_ainf;
    auto u_RP = u_PA + w_a;
    auto v_RP = -GetBody()->ProjectVectorInBody(inflowRelativeVelocityInWorld, NWU).GetVy();

    auto r_RP = 0.5 * Dp * std::sqrt(std::abs(u0 / u_RP));

    auto kd = std::pow(std::abs(u_RA / u_RP), 2. * std::pow(2. / (2. + std::sqrt(MU_PI) * r_RP / (2 * br)), 8));

    auto A_RP = 2. * r_RP / m_rudderForce->GetHeight() * A_R;
    auto A_RA = A_R - A_RP;

    // Outside slipstream rudder force
    m_rudderForce->SetProjectedLateralArea(A_RA);
    auto outsideSlipstreamRudderForce = m_rudderForce->ComputeGeneralizedForceInWorld(inflowRelativeVelocityInWorld);

    // Inside slipstream rudder force
    m_rudderForce->SetProjectedLateralArea(A_RP);
    auto inflowPropellerSlipstream = GetBody()->ProjectVectorInBody(Velocity(-u_RP, -v_RP, 0.), NWU);
    auto insideSlipstreamRudderForce = m_rudderForce->ComputeGeneralizedForceInWorld(inflowPropellerSlipstream);

    // Re-set of total rudder area
    m_rudderForce->SetProjectedLateralArea(A_R);

    auto propellerTorqueAtRudder = GetTorqueInWorldAtPointInBody(m_rudderForce->GetPositionInBody(), NWU);

    auto totalTorqueAtRudder = propellerTorqueAtRudder + kd * insideSlipstreamRudderForce.GetTorque() + outsideSlipstreamRudderForce.GetTorque();
    auto totalForce = propellerForce.GetForce() + kd*insideSlipstreamRudderForce.GetForce() + outsideSlipstreamRudderForce.GetForce();

    SetForceTorqueInWorldAtPointInBody(totalForce, totalTorqueAtRudder, m_rudderForce->GetPositionInBody(), NWU);

  }

  void FrPropellerRudder::Initialize() {
    FrForce::Initialize();
    m_propellerForce->Initialize();
    m_rudderForce->Initialize();

    m_longitudinalDistancePropellerRudder =
        m_propellerForce->GetPositionInBody().GetX() - m_rudderForce->GetPositionInBody().GetX();
  }

  std::shared_ptr<FrPropellerRudder> make_propeller_rudder(const std::string &name, FrBody *body) {
    auto force = std::make_shared<FrPropellerRudder>(name, body);
    body->AddExternalForce(force);
    return force;
  }

} // end namespace frydom