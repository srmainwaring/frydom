// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================

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

  top_plate->SetFixedInWorld(true);

  auto node_A = top_plate->NewNode("node_A");
  node_A->SetPositionInBody({0., 0., -325}, NWU);


  // Mobile nodes
  auto node_C = top_plate->NewNode("node_C");
  node_C->SetPositionInBody({0., 0., 0.}, 120, 24.8, 1, DEG, NED);

  auto node_D = top_plate->NewNode("node_D");
  node_D->SetPositionInBody({0., 0., 0.}, 90, 24.8, -1., DEG, NWU);

  auto node_E = top_plate->NewNode("node_E");
  node_E->SetPositionInBody({0., 0., 0.}, 60, 24.8, 1, DEG, NED);



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


  // cable properties

  auto props = make_cable_properties();
  props->SetLinearDensity(30.);
  props->SetDiameter(0.05);
  props->SetYoungModulus(1e9);


  double tendons_length = 44;

//  auto tendon_1 = make_catenary_line("tendon_1", node_F, node_C, props, true, tendons_length, AIR);
//  auto tendon_2 = make_catenary_line("tendon_2", node_G, node_D, props, true, tendons_length, AIR);
//  auto tendon_3 = make_catenary_line("tendon_3", node_H, node_E, props, true, tendons_length, AIR);


  auto tendon_1 = make_fea_cable("tendon_1", node_F, node_C, props, tendons_length, 10);



  system.RunInViewer();


  return 0;
}
