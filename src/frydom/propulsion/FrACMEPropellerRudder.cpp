//
// Created by lletourn on 03/09/2021.
//

#include <frydom/utils/FrFileSystem.h>
#include "FrACMEPropellerRudder.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/environment/FrEnvironment.h"

using namespace acme;

namespace frydom {

  FrACMEPropellerRudder::FrACMEPropellerRudder(const std::string &name,
                                               PropellerModelType prop_type,
                                               const std::shared_ptr<FrNode> &propeller_node,
                                               PropellerParams prop_params,
                                               const std::string &prop_perf_data_string,
                                               RudderModelType rudder_type,
                                               const std::shared_ptr<FrNode> &rudder_node,
                                               RudderParams rudder_params,
                                               const std::string &rudder_perf_data_string) :
      FrActuatorForceBase(name, "ACMEPropellerRudder", propeller_node->GetBody()),
      m_propeller_node(propeller_node), m_rudder_node(rudder_node) {

    switch (prop_type) {
      case E_FPP1Q:
        switch (rudder_type) {
          case E_SIMPLE_RUDDER:
            m_acme_propeller_rudder = std::make_unique<PropellerRudder<FPP1Q, SimpleRudderModel>>(prop_params,
                                                                                                  prop_perf_data_string,
                                                                                                  rudder_params,
                                                                                                  rudder_perf_data_string);

            break;
          case E_FLAP_RUDDER:
            m_acme_propeller_rudder = std::make_unique<PropellerRudder<FPP1Q, FlapRudderModel>>(prop_params,
                                                                                                prop_perf_data_string,
                                                                                                rudder_params,
                                                                                                rudder_perf_data_string);
            break;
        }
        break;
      case E_FPP4Q:
        switch (rudder_type) {
          case E_SIMPLE_RUDDER:
            m_acme_propeller_rudder = std::make_unique<PropellerRudder<FPP4Q, SimpleRudderModel>>(prop_params,
                                                                                                  prop_perf_data_string,
                                                                                                  rudder_params,
                                                                                                  rudder_perf_data_string);
            break;
          case E_FLAP_RUDDER:
            m_acme_propeller_rudder = std::make_unique<PropellerRudder<FPP4Q, FlapRudderModel>>(prop_params,
                                                                                                prop_perf_data_string,
                                                                                                rudder_params,
                                                                                                rudder_perf_data_string);
            break;
        }
        break;
      case E_CPP:
        switch (rudder_type) {
          case E_SIMPLE_RUDDER:
            m_acme_propeller_rudder = std::make_unique<PropellerRudder<CPP, SimpleRudderModel>>(prop_params,
                                                                                                prop_perf_data_string,
                                                                                                rudder_params,
                                                                                                rudder_perf_data_string);
            break;
          case E_FLAP_RUDDER:
            m_acme_propeller_rudder = std::make_unique<PropellerRudder<CPP, FlapRudderModel>>(prop_params,
                                                                                              prop_perf_data_string,
                                                                                              rudder_params,
                                                                                              rudder_perf_data_string);
            break;
        }
        break;
    }
  }

  void FrACMEPropellerRudder::Initialize() {

    c_water_density = GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
    c_xr = m_propeller_node->GetNodePositionInBody(NWU).GetX() - m_rudder_node->GetNodePositionInBody(NWU).GetX();

    m_acme_propeller_rudder->Initialize();

//    //TODO : remove when command function is moved in VSL
//    if (m_rudderAngleFunction == NULL)
//      SetRudderCommandAngle(GetRudderAngle(DEG), DEG);

    FrForce::Initialize();
  }

  void FrACMEPropellerRudder::Compute(double time) {

    // get fluid relative velocity at propeller position, in propeller reference frame
    Velocity relative_node_velocity = -GetSystem()->GetEnvironment()->GetRelativeVelocityInFrame(
        m_propeller_node->GetFrameInWorld(), m_propeller_node->GetVelocityInWorld(NWU), WATER, NWU);

    auto uP0 = relative_node_velocity.GetVx();
    auto vP0 = relative_node_velocity.GetVy();

    auto r = GetBody()->GetAngularVelocityInWorld(NWU).GetWz();

    m_acme_propeller_rudder->Compute(c_water_density, uP0, vP0, r, c_xr, m_rpm, m_pitch_ratio, m_rudder_angle_deg);

    auto Fx = m_acme_propeller_rudder->GetPropellerRudderFx();
    auto Fy = m_acme_propeller_rudder->GetPropellerRudderFy();
    auto Mz = m_acme_propeller_rudder->GetPropellerRudderMz();

    auto force = m_propeller_node->ProjectVectorInWorld(Force(Fx, Fy, 0.), NWU);
    auto torque = m_propeller_node->ProjectVectorInWorld(Torque(m_acme_propeller_rudder->GetPropellerTorque(), 0., Mz),
                                                         NWU);

    SetForceTorqueInWorldAtPointInBody(force, torque, m_propeller_node->GetNodePositionInBody(NWU), NWU);

  }

  void FrACMEPropellerRudder::DefineLogMessages() {

    auto msg = NewMessage("FrRudderForce", "Rudder force message");

    msg->AddField<double>("Time", "s", "Current time of the simulation",
                          [this]() { return m_chronoForce->GetChTime(); });

    msg->AddField<double>("RudderAngle", "rad", "Rudder angle",
                          [this]() { return m_rudder_angle_deg * DEG2RAD; });

    msg->AddField<double>("RotationalVelocity", "rad/s", "Rotational velocity",
                          [this]() { return mathutils::convert_frequency(m_rpm, RPM, RADS); });

    msg->AddField<double>("Thrust", "N", "Thrust delivered by the propeller",
                          [this]() { return m_acme_propeller_rudder->GetPropellerThrust(); });

    msg->AddField<double>("Torque", "Nm", "Torque delivered by the propeller",
                          [this]() { return m_acme_propeller_rudder->GetPropellerTorque(); });

    msg->AddField<double>("Power", "W", "Power delivered by the propeller",
                          [this]() { return m_acme_propeller_rudder->GetPropellerPower(); });

    msg->AddField<double>("Efficiency", "", "Efficiency delivered by the propeller",
                          [this]() { return m_acme_propeller_rudder->GetPropellerEfficiency(); });

//    msg->AddField<double>("uP0", "m/s", "Longitudinal velocity at the propeller position, in body reference frame",
//                          [this]() { return c_uP0; });

//    msg->AddField<double>("u_R0", "m/s", "Rudder longitudinal relative velocity, in body reference frame",
//                          [this]() { return c_uR0; });
//
//    msg->AddField<double>("v_R0", "m/s", "Rudder transversal relative velocity, in body reference frame",
//                          [this]() { return c_vR0; });

//    msg->AddField<double>("DriftAngle", "rad", "Drift angle",
//                          [this]() { return m_acme_propeller_rudder->GetDriftAngle(RAD); });
//
//    msg->AddField<double>("AttackAngle", "rad", "Attack angle",
//                          [this]() { return m_acme_rudder->GetAttackAngle(RAD); });

//    msg->AddField<double>("Drag", "N", "Drag delivered by the rudder",
//                          [this]() { return m_acme_propeller_rudder->GetRudderDrag(); });
//
//    msg->AddField<double>("Lift", "N", "Lift delivered by the rudder",
//                          [this]() { return m_acme_propeller_rudder->GetRudderLift(); });
//
//    msg->AddField<double>("Torque", "Nm", "Torque delivered by the rudder",
//                          [this]() { return m_acme_propeller_rudder->GetRudderMz(); });

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

  std::shared_ptr<FrACMEPropellerRudder>
  make_ACME_propeller_rudder(const std::string &name,
                             PropellerModelType prop_type,
                             const std::shared_ptr<FrNode> &propeller_node,
                             PropellerParams prop_params,
                             const std::string &prop_input_filepath,
                             RudderModelType rudder_type,
                             const std::shared_ptr<FrNode> &rudder_node,
                             RudderParams rudder_params,
                             const std::string &rudder_input_filepath) {

    std::string rudder_tmp_string = rudder_input_filepath;

    if (!FrFileSystem::isfile(rudder_input_filepath)) {
      std::cerr << "make_ACME_propeller_rudder : rudder_input_filepath is a filepath to a non existent file : " +
                   rudder_input_filepath << std::endl;
      exit(1);
    }
    std::ifstream tmp_buffer(rudder_input_filepath);
    json node = json::parse(tmp_buffer);
    rudder_tmp_string = node["rudder"]["load_coefficients"].dump();

    std::string prop_tmp_string = prop_input_filepath;

    if (!FrFileSystem::isfile(prop_input_filepath)) {
      std::cerr << "make_ACME_propeller_rudder : prop_input_filepath is a filepath to a non existent file : " +
                   prop_input_filepath << std::endl;
      exit(1);
    }
    tmp_buffer = std::ifstream(prop_input_filepath);
    node = json::parse(tmp_buffer);
    prop_tmp_string = node["propeller"]["open_water_table"].dump();

    auto prop_rudder = std::make_shared<FrACMEPropellerRudder>(name,
                                                               prop_type,
                                                               propeller_node,
                                                               prop_params,
                                                               prop_tmp_string,
                                                               rudder_type,
                                                               rudder_node,
                                                               rudder_params,
                                                               rudder_tmp_string);
    propeller_node->GetBody()->AddExternalForce(prop_rudder);
    return prop_rudder;
  }
}