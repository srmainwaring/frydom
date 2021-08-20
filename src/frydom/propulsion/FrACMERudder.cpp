//
// Created by lletourn on 20/08/2021.
//

#include "FrACMERudder.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/core/common/FrNode.h"

#include "acme/rudder/FlapRudderModel.h"

namespace frydom {

  FrACMERudder::FrACMERudder(const std::string &name, const frydom::FrNode &rudder_node,
                             frydom::RudderParams params, const std::string &perf_data_json_string,
                             frydom::RudderType type) :
      FrActuatorForceBase(name, "FrACMERudder", rudder_node.GetBody()) {

    switch (type) {
      case acme::RudderModelType::E_SIMPLE_RUDDER :
        m_acme_rudder = std::make_unique<acme::SimpleRudderModel>(params, perf_data_json_string);
        break;
      case acme::RudderModelType::E_FLAP_RUDDER :
        m_acme_rudder = std::make_unique<acme::FlapRudderModel>(params, perf_data_json_string);
        break;

    }
  }

  void FrACMERudder::SetRudderAngle(double rudder_angle) {
    m_rudder_angle = rudder_angle;
  }

  void FrACMERudder::Initialize() {

    m_acme_rudder->Initialize();

    c_water_density = GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

    FrForce::Initialize();
  }

  void FrACMERudder::Compute(double time) {

    // get fluid relative velocity at propeller position, in propeller reference frame
    Velocity relative_node_velocity = -GetSystem()->GetEnvironment()->GetRelativeVelocityInFrame(
        m_node->GetFrameInWorld(), m_node->GetVelocityInWorld(NWU), WATER, NWU);

    c_uR0 = relative_node_velocity.GetVx();
    c_vR0 = relative_node_velocity.GetVy();

    // compute propeller loads using acme
    m_acme_rudder->Compute(c_water_density, c_uR0, c_vR0, m_rudder_angle);

    // project thrust and torque loads in world reference frame
    auto force = m_node->ProjectVectorInWorld(Force(m_acme_rudder->GetFx(), m_acme_rudder->GetFy(), 0.), NWU);
    auto torque = m_node->ProjectVectorInWorld(Torque(0., 0., m_acme_rudder->GetMz()), NWU);

    SetForceTorqueInWorldAtPointInBody(force, torque, m_node->GetNodePositionInBody(NWU), NWU);
  }

  void FrACMERudder::DefineLogMessages() {

    auto msg = NewMessage("FrRudderForce", "Rudder force message");

    msg->AddField<double>("Time", "s", "Current time of the simulation",
                          [this]() { return m_chronoForce->GetChTime(); });

    msg->AddField<double>("RudderAngle", "rad", "Rudder angle",
                          [this]() { return m_rudder_angle; });

    msg->AddField<double>("DriftAngle", "rad", "Drift angle",
                          [this]() { return m_acme_rudder->GetDriftAngle(RAD); });

    msg->AddField<double>("AttackAngle", "rad", "Attack angle",
                          [this]() { return m_acme_rudder->GetAttackAngle(RAD); });

    msg->AddField<double>("Drag", "N", "Drag delivered by the rudder",
                          [this]() { return m_acme_rudder->GetDrag(); });

    msg->AddField<double>("Lift", "N", "Lift delivered by the rudder",
                          [this]() { return m_acme_rudder->GetLift(); });

    msg->AddField<double>("Torque", "Nm", "Torque delivered by the rudder",
                          [this]() { return m_acme_rudder->GetTorque(); });

    msg->AddField<double>("u_R0", "m/s", "Rudder longitudinal relative velocity, in body reference frame",
                          [this]() { return c_uR0; });

    msg->AddField<double>("v_R0", "m/s", "Rudder transversal relative velocity, in body reference frame",
                          [this]() { return c_vR0; });

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

  std::shared_ptr<FrACMERudder>
  make_ACME_rudder(const std::string &name, const FrNode &rudder_node, RudderParams params,
                   const std::string &perf_data_json_string, RudderType type) {
    auto force = std::make_shared<FrACMERudder>(name, rudder_node, params, perf_data_json_string, type);
    rudder_node.GetBody()->AddExternalForce(force);
    return force;
  }

  std::shared_ptr<FrACMERudder>
  make_ACME_rudder(const std::string &name, const FrNode &rudder_node, double area_m2, double chord_m,
                           double height_m, double wake_fraction, const std::string &perf_data_json_string,
                           RudderType type) {
    RudderParams params;
    params.m_lateral_area_m2 = area_m2;
    params.m_chord_m = chord_m;
    params.m_height_m = height_m;
    params.m_hull_wake_fraction_0 = wake_fraction;
    return make_ACME_rudder(name, rudder_node, params, perf_data_json_string, type);
  }
}