//
// Created by lletourn on 20/08/2021.
//

#include "FrACMEPropeller.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/core/common/FrNode.h"
#include "acme/propeller/FPP1Q.h"
#include "acme/propeller/FPP4Q.h"
#include "acme/propeller/CPP.h"


namespace frydom {

  FrACMEPropeller::FrACMEPropeller(const std::string &name, const std::shared_ptr<FrNode> &propeller_node,
                                   PropellerParams params, const std::string &perf_data_json_string,
                                   PropellerType type) :
      m_node(propeller_node), FrActuatorForceBase(name, "FrACMEPropeller", propeller_node->GetBody()) {

    switch (type) {
      case acme::PropellerModelType::E_FPP1Q :
        m_acme_propeller = std::make_unique<acme::FPP1Q>(params, perf_data_json_string);
        break;
      case acme::PropellerModelType::E_FPP4Q :
        m_acme_propeller = std::make_unique<acme::FPP4Q>(params, perf_data_json_string);
        break;
      case acme::PropellerModelType::E_CPP :
        m_acme_propeller = std::make_unique<acme::CPP>(params, perf_data_json_string);
        break;
    }

  }

  void FrACMEPropeller::SetRPM(double rpm) {
    m_rpm = rpm;
  }

  void FrACMEPropeller::SetPitchRatio(double pitch_ratio) {
    m_pitch_ratio = pitch_ratio;
  }


  void FrACMEPropeller::Initialize() {

    m_acme_propeller->Initialize();

    c_water_density = GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

    FrForce::Initialize();
  }

  void FrACMEPropeller::Compute(double time) {

    // get fluid relative velocity at propeller position, in propeller reference frame
    Velocity relative_node_velocity = -GetSystem()->GetEnvironment()->GetRelativeVelocityInFrame(
        m_node->GetFrameInWorld(), m_node->GetVelocityInWorld(NWU), WATER, NWU);

    c_uP0 = relative_node_velocity.GetVx();
    c_vP0 = relative_node_velocity.GetVy();

    // compute propeller loads using acme
    m_acme_propeller->Compute(c_water_density, c_uP0, c_vP0, m_rpm, m_pitch_ratio);

    // project thrust and torque loads in world reference frame
    auto force = m_node->ProjectVectorInWorld(Force(m_acme_propeller->GetThrust(), 0., 0.), NWU);
    auto torque = m_node->ProjectVectorInWorld(Torque(m_acme_propeller->GetTorque(), 0., 0.), NWU);

    SetForceTorqueInWorldAtPointInBody(force, torque, m_node->GetNodePositionInBody(NWU), NWU);

  }

  void FrACMEPropeller::DefineLogMessages() {
    auto msg = NewMessage("FrForce", "Force message");

    msg->AddField<double>("Time", "s", "Current time of the simulation",
                          [this]() { return m_chronoForce->GetChTime(); });

    msg->AddField<double>("Thrust", "N", "Thrust delivered by the propeller",
                          [this]() { return m_acme_propeller->GetThrust(); });

    msg->AddField<double>("Torque", "Nm", "Torque delivered by the propeller",
                          [this]() { return m_acme_propeller->GetTorque(); });

    msg->AddField<double>("Power", "W", "Power delivered by the propeller",
                          [this]() { return m_acme_propeller->GetPower(); });

    msg->AddField<double>("uP0", "m/s", "Longitudinal velocity at the propeller position, in body reference frame",
                          [this]() { return c_uP0; });

    msg->AddField<double>("RotationalVelocity", "rad/s", "Rotational velocity",
                          [this]() { return mathutils::convert_frequency(m_rpm, RPM, RADS); });

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

  std::shared_ptr<FrACMEPropeller>
  make_ACME_propeller(const std::string &name, const std::shared_ptr<FrNode> &propeller_node, PropellerParams params,
                      const std::string &perf_data_json_string, PropellerType type) {
    auto force = std::make_shared<FrACMEPropeller>(name, propeller_node, params, perf_data_json_string, type);
    propeller_node->GetBody()->AddExternalForce(force);
    return force;
  }

  std::shared_ptr<FrACMEPropeller>
  make_ACME_propeller(const std::string &name, const std::shared_ptr<FrNode> &propeller_node, double diameter,
                      double wake_fraction, double thrust_deduction_factor, const std::string &screwDirection,
                      const std::string &perf_data_json_string, PropellerType type) {
    acme::SCREW_DIRECTION sc;
    if (screwDirection != "LEFT_HANDED" or screwDirection != "RIGHT_HANDED")
      throw std::runtime_error("wrong screwDirection given : LEFT_HANDED or RIGHT_HANDED only");
    sc = screwDirection == "LEFT_HANDED" ? acme::SCREW_DIRECTION::LEFT_HANDED : acme::SCREW_DIRECTION::RIGHT_HANDED;
    PropellerParams params(diameter, wake_fraction, thrust_deduction_factor, sc);
    return make_ACME_propeller(name, propeller_node, params, perf_data_json_string, type);
  }
}