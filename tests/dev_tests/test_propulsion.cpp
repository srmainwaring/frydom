//
// Created by lletourn on 01/06/2021.
//

#include "frydom/frydom.h"
#include "acme/acme.h"

using namespace frydom;

int main() {

  FrOffshoreSystem system("test_propulsion");
  auto rho = 1025;
  system.GetEnvironment()->GetOcean()->SetDensity(rho);

  system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
  auto current = system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform();
//  current->SetNorth(1.0, MS, COMEFROM);
  current->Set(20, 1, DEG, MS, NWU, COMEFROM);

  auto body = system.NewBody("body");
  body->SetFixedInWorld(true);
//  body->SetVelocityInBodyNoRotation(Velocity(-1.0, 0., 0.), NWU);

  auto node = body->NewNode("node");

  // FPP_1Q
  std::string open_water_data_table = R"({"j":[0,0.5,1], "kt":[1,0.5,0], "kq":[0.5,1,0.5]})";

  json FPP1Q_json = { {"propeller", {
    {"reference","test"},
    {"type","FPP_FIRST_QUADRANT"},
    {"open_water_table",{
        {"screw_direction","RIGHT_HANDED"},
        {"J",{0.,0.5,1.}},
        {"kt",{1.,0.5,0.}},
        {"kq",{0.5,1.,0.5}},
    }}
  }}
  };

  std::string filename = "temp.json";
  std::ofstream file;
  file.open(filename, std::ios::trunc);
  file << FPP1Q_json.dump(2);
  file.close();

  double diameter_m = 2;
  double hull_wake_fraction_0 = 0.25;
  double thrust_deduction_factor_0 = 0.2;
  double rpm = 100;
//  bool use_advance_velocity_correction_factor = false;
//  double propeller_design_rpm;
//  double vessel_design_speed_ms;

  Velocity prop_relative_velocity = - system.GetEnvironment()->GetRelativeVelocityInFrame(node->GetFrameInWorld(),
                                                                                    node->GetVelocityInWorld(NWU),
                                                                                    WATER,
                                                                                    NWU);
  auto u_NWU = prop_relative_velocity.GetVx();
  auto v_NWU = prop_relative_velocity.GetVy();

//  auto file = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx027/propeller_FPP.json"});
  auto frydom_FPP1Q = make_first_quadrant_propeller_force("FPP1Q", body, node->GetNodePositionInBody(NWU), filename, NWU);
  frydom_FPP1Q->SetDiameter(diameter_m);
  frydom_FPP1Q->SetScrewDirection(SCREW_DIRECTION::RIGHT_HANDED);
  frydom_FPP1Q->SetRotationalVelocity(rpm,RPM);
  frydom_FPP1Q->SetStraightRunWakeFraction(hull_wake_fraction_0);
  frydom_FPP1Q->SetThrustDeductionFactor(thrust_deduction_factor_0);

  acme::ThrusterBaseParams params;
  params.m_diameter_m = diameter_m;
  params.m_screw_direction = acme::SCREW_DIRECTION::RIGHT_HANDED;
  params.m_hull_wake_fraction_0 = hull_wake_fraction_0;
  params.m_thrust_deduction_factor_0 = thrust_deduction_factor_0;
//  params.m_propeller_design_rpm = propeller_design_rpm;
//  params.m_vessel_design_speed_ms = vessel_design_speed_ms;

  auto acme_FPP1Q = acme::FPP1Q(params, open_water_data_table);

  system.Initialize();
  acme_FPP1Q.Initialize();
  acme_FPP1Q.Compute(rho, u_NWU, v_NWU, 100, 0.);
  std::cout<<"ACME Thrust :"<<acme_FPP1Q.GetThrust()<<std::endl;
  std::cout<<"ACME Torque :"<<acme_FPP1Q.GetTorque()<<std::endl;
  std::cout<<"ACME Power :"<<acme_FPP1Q.GetPower()<<std::endl;
  std::cout<<"ACME Efficiency :"<<acme_FPP1Q.GetPropellerEfficiency()<<std::endl;

  std::cout<< "FRyDoM Thrust :"<< frydom_FPP1Q->GetThrust()<<std::endl;
  std::cout<< "FRyDoM Torque :"<< frydom_FPP1Q->GetTorque()<<std::endl;
  std::cout<< "FRyDoM Power :"<< frydom_FPP1Q->GetPower()<<std::endl;


//  auto file = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx022/propeller_BCP_1320f.json"});
//  auto force = make_controllable_pitch_propeller("CPPForce", body.get(), Position(), file, NWU);

//  auto file = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx022/propeller_FPP_template.json"});
//  auto fpp = std::make_shared<FrFirstQuadrantPropellerForce>("FPP", body.get(), Position(), file, NWU);
//  body->AddExternalForce(fpp);

//  auto file = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx022/flap_rudder.json"});
//  auto rudder = make_flap_rudder_force("rudder", body, node, file);
//  rudder->SetFlapLaw(0.8);
//  rudder->SetRampSlope(1, DEGS);
//  rudder->SetProjectedLateralArea(15.061);
//
//  rudder->SetRudderAngle(20, DEG);

  system.Initialize();

  system.AdvanceTo(30);

//  force->SetPitchRatio(1.5);
//  std::cout<<"Ct="<<force->Ct(-180*DEG2RAD)<<", Cq="<<force->Cq(-180*DEG2RAD)<<std::endl;

//  std::cout << "kt = " << fpp->kt(1.) << ", kq = " << fpp->kq(1.) << std::endl;
//
//  auto inflowVelocity = rudder->GetRudderRelativeVelocityInWorld();
//  auto attack_angle = rudder->GetAttackAngle(-inflowVelocity);
//  std::cout << "rudder inflow velocity : " << inflowVelocity << std::endl;
//  std::cout << "rudder angle : " << rudder->GetRudderAngle(DEG) << std::endl;
//  std::cout << "rudder drift angle : " << rudder->GetDriftAngle(-inflowVelocity) * RAD2DEG << std::endl;
//  std::cout << "rudder attack angle : " << attack_angle * RAD2DEG << std::endl;
//
//
//  std::cout << "cd = "    << rudder->GetDragCoefficient(attack_angle)
//            << ", cl = "  << rudder->GetLiftCoefficient(attack_angle)
//            << ", cn = "  << rudder->GetTorqueCoefficient(attack_angle) << std::endl;
//
//  std::cout << "rudder force : " << rudder->GetForceInWorld(NWU) << std::endl;
//  std::cout << "rudder torque : " << rudder->GetTorqueInWorldAtCOG(NWU) << std::endl;

//  auto X = force->GetCoeff().GetX();
//  std::cout<<"X : "<<Eigen::VectorXd::Map(&X[0], X.size())<<std::endl;
//  auto Y = force->GetCoeff().GetY();
//  std::cout<<"Y : "<<Eigen::VectorXd::Map(&Y[0], Y.size())<<std::endl;



}