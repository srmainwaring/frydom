//
// Created by lletourn on 20/08/2021.
//

#include "FrACMERudder.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/core/common/FrNode.h"

#include "acme/rudder/FlapRudderModel.h"

#include "frydom/utils/FrFileSystem.h"

namespace frydom {

  FrACMERudder::FrACMERudder(const std::string &name, const std::shared_ptr<FrNode> &rudder_node,
                             RudderParams params, const std::string &perf_data_json_string,
                             RudderType type) :
      m_node(rudder_node), FrActuatorForceBase(name, "FrACMERudder", rudder_node->GetBody()) {

    switch (type) {
      case acme::RudderModelType::E_SIMPLE_RUDDER :
        m_acme_rudder = std::make_unique<acme::SimpleRudderModel>(params, perf_data_json_string);
        break;
      case acme::RudderModelType::E_FLAP_RUDDER :
        m_acme_rudder = std::make_unique<acme::FlapRudderModel>(params, perf_data_json_string);
        break;
    }
  }

  void FrACMERudder::SetRudderAngle(double rudder_angle, ANGLE_UNIT unit) {
    m_rudder_angle_deg = rudder_angle;
    if (unit == RAD) m_rudder_angle_deg *= RAD2DEG;
  }

  double FrACMERudder::GetRudderAngle(ANGLE_UNIT unit) const {
    double angle = m_rudder_angle_deg;
    if (unit == RAD) angle *= DEG2RAD;
    return angle;
  }

  void FrACMERudder::SetRudderCommandAngle(double angle, ANGLE_UNIT unit) {
    if (unit == RAD) angle *= RAD2DEG;

    double actualRudderAngle = GetRudderAngle(DEG);

    if (std::abs(angle - GetRudderAngle(DEG)) <= 0)
      m_rudderAngleFunction = new FrConstantFunction(angle);
    else {
      auto t_ramp = std::abs(angle - actualRudderAngle) / m_ramp_slope;
      m_rudderAngleFunction = new FrLinearRampFunction(GetSystem()->GetTime(), actualRudderAngle,
                                                       GetSystem()->GetTime() + t_ramp, angle);
    }

  }

  void FrACMERudder::Initialize() {

    m_acme_rudder->Initialize();
    m_acme_rudder->Log(true);

    c_x_gr = GetBody()->GetCOG(NWU).GetX() - m_node->GetNodePositionInBody(NWU).GetX();

    c_water_density = GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

    //TODO : remove when command function is moved in VSL
    if (m_rudderAngleFunction == NULL)
      SetRudderCommandAngle(GetRudderAngle(DEG), DEG);

    FrForce::Initialize();
  }

  void FrACMERudder::Compute(double time) {

    //TODO : remove when command function is moved in VSL
    SetRudderAngle(m_rudderAngleFunction->Get_y(time), DEG);

    // ship velocity at COG
    auto ship_vel = GetBody()->GetCOGLinearVelocityInWorld(NWU);
    auto ship_frame = GetBody()->GetFrameAtCOG();
    Velocity ship_relative_velocity = -GetSystem()->GetEnvironment()->GetRelativeVelocityInFrame(ship_frame, ship_vel, WATER, NWU);

    auto u = ship_relative_velocity.GetVx();
    auto v = ship_relative_velocity.GetVy();

    auto r = GetBody()->GetAngularVelocityInWorld(NWU).GetWz();

    // get fluid relative velocity at propeller position, in propeller reference frame
    Velocity relative_node_velocity = -GetSystem()->GetEnvironment()->GetRelativeVelocityInFrame(
        m_node->GetFrameInWorld(), m_node->GetVelocityInWorld(NWU), WATER, NWU);

    c_uR0 = relative_node_velocity.GetVx();
    c_vR0 = relative_node_velocity.GetVy();

    if (relative_node_velocity.norm() == 0.)
      return;

    // compute propeller loads using acme
    m_acme_rudder->Compute(c_water_density, c_uR0, c_vR0, m_rudder_angle_deg, u, v, r, c_x_gr);

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
                          [this]() { return m_rudder_angle_deg * DEG2RAD; });

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

  void FrACMERudder::StepFinalize() {
    m_acme_rudder->Finalize(GetSystem()->GetTime());
  }

  std::shared_ptr<FrACMERudder>
  make_ACME_rudder(const std::string &name, const std::shared_ptr<FrNode> &rudder_node, RudderParams params,
                   const std::string &rudder_input_filepath, RudderType type) {

    std::string tmp_string = rudder_input_filepath;

    if (!FrFileSystem::isfile(rudder_input_filepath)) {
      std::cerr << "make_ACME_rudder : rudder_input_filepath is a filepath to a non existent file : " +
                   rudder_input_filepath << std::endl;
      exit(1);
    }

    std::ifstream tmp_buffer(rudder_input_filepath);
    json node = json::parse(tmp_buffer);
    tmp_string = node["rudder"]["load_coefficients"].dump();

    auto force = std::make_shared<FrACMERudder>(name, rudder_node, params, tmp_string, type);
    rudder_node->GetBody()->AddExternalForce(force);
    return force;
  }
}