//
// Created by lletourn on 01/06/2021.
//

#include "frydom/frydom.h"

using namespace frydom;

int main() {

  FrOffshoreSystem system("test_FrCPPForce");

  auto body = system.NewBody("body");

//  auto file = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx022/propeller_BCP_1320f.json"});
//  auto force = std::make_shared<FrCPPForce>("CPPForce", body.get(), Position(), file);
//  body->AddExternalForce(force);

  auto file = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx022/propeller_FPP_template.json"});
  auto fpp = std::make_shared<FrFirstQuadrantPropellerForce>("FPP", body.get(), Position(), file);
  body->AddExternalForce(fpp);

  system.Initialize();

//  std::cout<<"Ct="<<force->Ct(180*DEG2RAD, 1.4)<<", Cq="<<force->Cq(180*DEG2RAD, 1.4)<<std::endl;

  std::cout<<"kt = "<<fpp->kt(1.)<<"kq = "<<fpp->kq(1.)<<std::endl;

//  auto X = force->GetCoeff().GetX();
//  std::cout<<"X : "<<Eigen::VectorXd::Map(&X[0], X.size())<<std::endl;
//  auto Y = force->GetCoeff().GetY();
//  std::cout<<"Y : "<<Eigen::VectorXd::Map(&Y[0], Y.size())<<std::endl;



}