//
// Created by frongere on 11/06/2020.
//

#include "frydom/frydom.h"


using namespace frydom;

int main() {

  std::cout << "Demo tensegrity" << std::endl;

  FrOffshoreSystem system("tensegrity");

  system.GetEnvironment()->GetOcean()->ShowFreeSurface(false);




  double material_density = 700.;

  auto top_plate = system.NewBody("top_plate");
  makeItCylinder(top_plate, 12.5, 2, MU_PI * 12.5 * 12.4 * 2 * material_density);
  top_plate->RotateAroundCOG(FrRotation({1., 0., 0.}, 90*DEG2RAD, NWU), NWU);
  top_plate->SetPosition({0., 0., 45.}, NWU);

  auto node_A = top_plate->NewNode("node_A");
  node_A->SetPositionInBody({0., 0., -325}, NWU);


  auto node_C = top_plate->NewNode("node_C");
//  node_C->SetPositionInBody({0., 0., 0.}, 120., 248, 1, )


  // Fixed nodes
  auto world_body = system.GetWorldBody();
  auto node_B = world_body->NewNode("node_B");
  node_B->SetPositionInWorld({0., 0., 325}, NWU);

  auto node_F = world_body->NewNode("node_F");
  node_F->SetPositionInWorld({0., 0., 0.}, 120, 24.8, -1., DEG, NED);

  auto node_G = world_body->NewNode("node_G");
  node_G->SetPositionInWorld({0., 0., 0.}, 90, 24.8, 1., DEG, NWU);

  auto node_H = world_body->NewNode("node_H");
  node_H->SetPositionInWorld({0., 0., 0.}, 60, 24.8, -1., DEG, NED);




  system.RunInViewer();


  return 0;
}
