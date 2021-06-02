//
// Created by lletourn on 01/06/2021.
//

#include "frydom/frydom.h"

using namespace frydom;

int main() {

  FrOffshoreSystem system("test_FrFlapRudderForce");

  auto body = system.NewBody("body");

  auto node = body->NewNode("rudderNode");

  auto rudderFile = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx022/flap_rudder.json"});
  auto force = std::make_shared<FrFlapRudderForce>("flapRudderForce", body.get(), rudderFile, node);

  system.Initialize();

  force->SetRudderAngle(-52/0.8 * DEG2RAD);
  std::cout<<"flap angle = "<<force->GetFlapAngle()<<std::endl;
  std::cout<<"coeffs="<<force->GetCoefficients(20. * DEG2RAD)<<std::endl;

//  auto X = force->GetCoeff().GetX();
//  std::cout<<"X : "<<Eigen::VectorXd::Map(&X[0], X.size())<<std::endl;
//  auto Y = force->GetCoeff().GetY();
//  std::cout<<"Y : "<<Eigen::VectorXd::Map(&Y[0], Y.size())<<std::endl;



}