//
// Created by lletourn on 01/06/2021.
//

#include "frydom/frydom.h"

using namespace frydom;

int main() {

  FrOffshoreSystem system("test_propulsion");

  auto body = system.NewBody("body");
  body->SetFixedInWorld(true);
//  body->SetVelocityInBodyNoRotation(Velocity(-1.0, 0., 0.), NWU);

  auto node = body->NewNode("node");

//  auto file = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx022/propeller_BCP_1320f.json"});
//  auto force = make_controllable_pitch_propeller("CPPForce", body.get(), Position(), file, NWU);

//  auto file = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx022/propeller_FPP_template.json"});
//  auto fpp = std::make_shared<FrFirstQuadrantPropellerForce>("FPP", body.get(), Position(), file, NWU);
//  body->AddExternalForce(fpp);

  auto file = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx022/flap_rudder.json"});
  auto rudder = make_flap_rudder_force("rudder", body, node, file);
  rudder->SetFlapLaw(0.8);
  rudder->SetRampSlope(1, DEGS);
  rudder->SetProjectedLateralArea(15.061);

  rudder->SetRudderAngle(20, DEG);

  system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
  auto current = system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform();
  current->SetNorth(1.0, MS, COMEFROM);
//  current->Set(20, 1, DEG, MS, NWU, COMEFROM);

  system.Initialize();

  system.AdvanceTo(30);

//  force->SetPitchRatio(1.5);
//  std::cout<<"Ct="<<force->Ct(-180*DEG2RAD)<<", Cq="<<force->Cq(-180*DEG2RAD)<<std::endl;

//  std::cout << "kt = " << fpp->kt(1.) << ", kq = " << fpp->kq(1.) << std::endl;
//
  auto inflowVelocity = rudder->GetRudderRelativeVelocityInWorld();
  auto attack_angle = rudder->GetAttackAngle(-inflowVelocity);
  std::cout << "rudder inflow velocity : " << inflowVelocity << std::endl;
  std::cout << "rudder angle : " << rudder->GetRudderAngle(DEG) << std::endl;
  std::cout << "rudder drift angle : " << rudder->GetDriftAngle(-inflowVelocity) * RAD2DEG << std::endl;
  std::cout << "rudder attack angle : " << attack_angle * RAD2DEG << std::endl;


  std::cout << "cd = "    << rudder->GetDragCoefficient(attack_angle)
            << ", cl = "  << rudder->GetLiftCoefficient(attack_angle)
            << ", cn = "  << rudder->GetTorqueCoefficient(attack_angle) << std::endl;

  std::cout << "rudder force : " << rudder->GetForceInWorld(NWU) << std::endl;
  std::cout << "rudder torque : " << rudder->GetTorqueInWorldAtCOG(NWU) << std::endl;

//  auto X = force->GetCoeff().GetX();
//  std::cout<<"X : "<<Eigen::VectorXd::Map(&X[0], X.size())<<std::endl;
//  auto Y = force->GetCoeff().GetY();
//  std::cout<<"Y : "<<Eigen::VectorXd::Map(&Y[0], Y.size())<<std::endl;



}