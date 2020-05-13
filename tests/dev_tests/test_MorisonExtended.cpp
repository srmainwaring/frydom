//
// Created by camille on 07/04/2020.
//

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {

  // System

  FrOffshoreSystem system("test_MorisonExtended");

  // Body

  auto body = system.NewBody("body");
  body->SetPosition({0., 0., 0.}, NWU);

  // Morison model

  auto morisonModel = make_morison_model("morisonModel", body);
  morisonModel->AddElement({0., -2, -10.}, {0., 2., -10.}, 1., 1., 0., 0.);
  morisonModel->SetExtendedModel(true);
  auto morisonForce = make_morison_force("morison", body, morisonModel);

  auto morisonNode = body->NewNode("morisonNode");
  morisonNode->SetPositionInBody({0., 0., -10}, NWU);

  // Link

  auto worldNode = system.GetWorldBody()->NewNode("worldNode");
  worldNode->SetPositionInWorld({0., 0., 0.}, NWU);
  auto bodyNode = body->NewNode("bodyNode");
  worldNode->RotateAroundXInWorld(M_PI_2, NWU);
  bodyNode->RotateAroundXInWorld(M_PI_2, NWU);

  auto link = make_revolute_link("revolute", &system, worldNode, bodyNode);
  auto motor = link->Motorize("motor", POSITION);
  auto x = new_var("x");
  motor->SetMotorFunction(M_PI_4 * sin(0.2*M_PI*x));

  // Simulation

  double dt = 0.01;
  double t_end = 50.;
  double time = 0.;

  system.SetTimeStep(dt);
  system.Initialize();

  while (time < t_end) {
    system.AdvanceTo(time);
    std::cout << "time : " << time <<  " s" << std::endl;
    time += dt;
  }




}