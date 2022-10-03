//
// Created by lletourn on 03/09/2021.
//

#include <frydom/utils/FrFileSystem.h>
#include "FrPropellerRudder.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/environment/FrEnvironment.h"

#include "acme/propeller_rudder/BrixPropellerRudder.h"
#include "acme/propeller_rudder/MMGPropellerRudder.h"

using namespace acme;

namespace frydom {

  FrPropellerRudder::FrPropellerRudder(const std::string &name,
                                       PropellerRudderModelType proprudder_type,
                                       PropellerModelType prop_type,
                                       const std::shared_ptr<FrNode> &propeller_node,
                                       PropellerParams prop_params,
                                       RudderModelType rudder_type,
                                       const std::shared_ptr<FrNode> &rudder_node,
                                       RudderParams rudder_params) :
      FrActuatorForceBase(name, "FrACMEPropellerRudder", propeller_node->GetBody()),
      m_propeller_node(propeller_node), m_rudder_node(rudder_node) {
    switch (proprudder_type) {
      case E_BRIX:
        m_acme_propeller_rudder = build_Brix_pr(prop_type, prop_params, rudder_type, rudder_params);
        break;
      case E_MMG:
        m_acme_propeller_rudder = build_MMG_pr(prop_type, prop_params, rudder_type, rudder_params);
        break;
    }
  }

  void FrPropellerRudder::Initialize() {

    c_water_density = GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
    c_x_pr = m_rudder_node->GetNodePositionInBody(NWU).GetX() - m_propeller_node->GetNodePositionInBody(NWU).GetX();
    c_x_gr = m_rudder_node->GetNodePositionInBody(NWU).GetX() - GetBody()->GetCOG(NWU).GetX();

    m_acme_propeller_rudder->Initialize();

    FrForce::Initialize();
  }

  void FrPropellerRudder::Compute(double time) {

    // ship velocity at COG
    auto ship_vel = GetBody()->GetCOGLinearVelocityInWorld(NWU);
    if (ship_vel.isZero())
      return;
    auto ship_frame = GetBody()->GetFrameAtCOG();
    Velocity ship_relative_velocity = -GetSystem()->GetEnvironment()->GetRelativeVelocityInFrame(ship_frame, ship_vel,
                                                                                                 WATER, NWU);

    auto u = ship_relative_velocity.GetVx();
    auto v = ship_relative_velocity.GetVy();

    // get fluid relative velocity at propeller position, in propeller reference frame
    Velocity relative_node_velocity = -GetSystem()->GetEnvironment()->GetRelativeVelocityInFrame(
        m_propeller_node->GetFrameInWorld(), m_propeller_node->GetVelocityInWorld(NWU), WATER, NWU);

    auto uP0 = relative_node_velocity.GetVx();
    auto vP0 = relative_node_velocity.GetVy();

    auto r = GetBody()->GetAngularVelocityInWorld(NWU).GetWz();

    m_acme_propeller_rudder->Compute(c_water_density, uP0, vP0, u, v, r, c_x_pr, c_x_gr, m_rpm, m_pitch_ratio,
                                     m_rudder_angle_deg);

    auto Fx = m_acme_propeller_rudder->GetPropellerRudderFx();
    auto Fy = m_acme_propeller_rudder->GetPropellerRudderFy();
    auto Mz = m_acme_propeller_rudder->GetPropellerRudderMz();

    auto force = m_propeller_node->ProjectVectorInWorld(Force(Fx, Fy, 0.), NWU);
    auto torque = m_propeller_node->ProjectVectorInWorld(Torque(m_acme_propeller_rudder->GetPropellerTorque(), 0., Mz),
                                                         NWU);

    SetForceTorqueInWorldAtPointInBody(force, torque, m_propeller_node->GetNodePositionInBody(NWU), NWU);

  }

  void FrPropellerRudder::DefineLogMessages() {


    // Propeller logs
    auto prop_msg = NewMessage("_propeller", "propeller force message");

    prop_msg->AddField<double>("Time", "s", "Current time of the simulation",
                               [this]() { return m_chronoForce->GetChTime(); });

    prop_msg->AddField<double>("RotationalVelocity", "rad/s", "Rotational velocity",
                               [this]() { return mathutils::convert_frequency(m_rpm, RPM, RADS); });

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
                                 [this]() { return GetRudderAngle(RAD); });

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

    m_acme_propeller_rudder->DefineLogMessages(prop_msg, rudder_msg);

  }

  void FrPropellerRudder::SetRPM(double rpm) {
    m_rpm = rpm;
  }

  void FrPropellerRudder::SetPitchRatio(double pitch_ratio) {
    m_pitch_ratio = pitch_ratio;
  }

  void FrPropellerRudder::SetRudderAngle(double rudder_angle, ANGLE_UNIT unit) {
    m_rudder_angle_deg = rudder_angle;
    if (unit == RAD) m_rudder_angle_deg *= RAD2DEG;
  }

  double FrPropellerRudder::GetRudderAngle(ANGLE_UNIT unit) const {
    double angle = m_rudder_angle_deg;
    if (unit == RAD) angle *= DEG2RAD;
    return angle;
  }

  double FrPropellerRudder::GetPropulsivePower() const {
    return m_acme_propeller_rudder->GetPropellerPower();
  }

  Force FrPropellerRudder::GetPropellerForceInWorld(FRAME_CONVENTION fc) const {
    auto thrust = m_acme_propeller_rudder->GetPropellerThrust();
    return m_propeller_node->ProjectVectorInWorld(Force(thrust, 0., 0.), fc);
  }

  Force FrPropellerRudder::GetPropellerForceInBody(FRAME_CONVENTION fc) const {
    return GetBody()->ProjectVectorInBody(GetPropellerForceInWorld(NWU), fc);
  }

  Torque FrPropellerRudder::GetPropellerTorqueInWorldAtPropeller(FRAME_CONVENTION fc) const {
    auto torque = m_acme_propeller_rudder->GetPropellerTorque();
    return m_propeller_node->ProjectVectorInWorld(Torque(torque, 0., 0.), fc);
  }

  Torque FrPropellerRudder::GetPropellerTorqueInBodyAtPropeller(FRAME_CONVENTION fc) const {
    return GetBody()->ProjectVectorInBody(GetPropellerTorqueInWorldAtPropeller(NWU), fc);
  }

  Torque FrPropellerRudder::GetPropellerTorqueInWorldAtCOG(FRAME_CONVENTION fc) const {
    Translation MG = m_propeller_node->GetPositionInWorld(fc) - GetBody()->GetCOGPositionInWorld(fc);
    return GetPropellerTorqueInWorldAtPropeller(fc) + MG.cross(GetPropellerForceInWorld(fc));
  }

  Torque FrPropellerRudder::GetPropellerTorqueInBodyAtCOG(FRAME_CONVENTION fc) const {
    return GetBody()->ProjectVectorInBody(GetPropellerTorqueInWorldAtCOG(NWU), fc);
  }

  Force FrPropellerRudder::GetRudderForceInWorld(FRAME_CONVENTION fc) const {
    auto rudderForceInNode = Force(m_acme_propeller_rudder->GetRudderFx(),
                                   m_acme_propeller_rudder->GetRudderFy(),
                                   0.);
    return m_rudder_node->ProjectVectorInWorld(rudderForceInNode, fc);
  }

  Force FrPropellerRudder::GetRudderForceInBody(FRAME_CONVENTION fc) const {
    return GetBody()->ProjectVectorInBody(GetRudderForceInWorld(NWU), fc);
  }

  Torque FrPropellerRudder::GetRudderTorqueInWorldAtRudder(FRAME_CONVENTION fc) const {
    return m_rudder_node->ProjectVectorInWorld(Torque(0., 0., m_acme_propeller_rudder->GetRudderMz()), fc);
  }

  Torque FrPropellerRudder::GetRudderTorqueInBodyAtRudder(FRAME_CONVENTION fc) const {
    return GetBody()->ProjectVectorInBody(GetRudderTorqueInWorldAtRudder(NWU), fc);
  }

  Torque FrPropellerRudder::GetRudderTorqueInWorldAtCOG(FRAME_CONVENTION fc) const {
    Translation MG = m_propeller_node->GetPositionInWorld(fc) - GetBody()->GetCOGPositionInWorld(fc);
    return GetRudderTorqueInWorldAtRudder(fc) + MG.cross(GetRudderForceInWorld(fc));
  }

  Torque FrPropellerRudder::GetRudderTorqueInBodyAtCOG(FRAME_CONVENTION fc) const {
    return GetBody()->ProjectVectorInBody(GetRudderTorqueInWorldAtCOG(NWU), fc);
  }

  std::shared_ptr<FrPropellerRudder>
  make_propeller_rudder_model(const std::string &name,
                              PropellerRudderModelType proprudder_type,
                              PropellerModelType prop_type,
                              const std::shared_ptr<FrNode> &propeller_node,
                              PropellerParams prop_params,
                              RudderModelType rudder_type,
                              const std::shared_ptr<FrNode> &rudder_node,
                              RudderParams rudder_params,
                              const std::string &prop_input_filepath,
                              const std::string &rudder_input_filepath) {

    if (FrFileSystem::isfile(prop_input_filepath)) {
      std::ifstream tmp_buffer = std::ifstream(prop_input_filepath);
      json node = json::parse(tmp_buffer);
      prop_params.m_thruster_perf_data_json_string = node["propeller"]["open_water_table"].dump();
    }

    if (FrFileSystem::isfile(rudder_input_filepath)) {
      std::ifstream tmp_buffer(rudder_input_filepath);
      json node = json::parse(tmp_buffer);
      rudder_params.m_perf_data_json_string = node["rudder"]["load_coefficients"].dump();
    }

    auto prop_rudder = std::make_shared<FrPropellerRudder>(name,
                                                           proprudder_type,
                                                           prop_type,
                                                           propeller_node,
                                                           prop_params,
                                                           rudder_type,
                                                           rudder_node,
                                                           rudder_params);
    propeller_node->GetBody()->AddExternalForce(prop_rudder);
    return prop_rudder;
  }
}