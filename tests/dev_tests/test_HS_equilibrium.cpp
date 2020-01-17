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


int main(int argc, char *argv[]) {

  std::cout << " ===================================================== \n"
               " Benchmark test : Nonlinear hydrostatics on a box \n"
               " ===================================================== \n" << std::endl;

  // -- System

  FrOffshoreSystem system("test_HS_equilibrium");

  auto Ocean = system.GetEnvironment()->GetOcean();
  Ocean->SetDensity(1023.);

  // -- Body

  auto body = system.NewBody("box");

  double L, B, H, c;
  L = H = B = 5.;
  c = 0.5;
//    L = 8; B = 4; H = 2; c = 0.5;

  auto mass = L * H * B * c * system.GetEnvironment()->GetFluidDensity(WATER);
  makeItBox(body, L, B, H, mass);

  body->TranslateInWorld(0,0,2.5,NWU);

  double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
  auto inertia = body->GetInertiaTensor();
  inertia.GetInertiaCoeffsAtCOG(Ixx, Iyy, Izz, Ixy, Ixz, Iyz, NWU);
  body->SetInertiaTensor(FrInertiaTensor(mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, Position(-2.5,-2.5,-2.5), NWU));

//  body->Rotate(FrRotation(Direction(1,0,0), 70*DEG2RAD, NWU));

  std::cout << "mass : "<< mass << std::endl;

//    body->RemoveAssets();
  auto boxMesh = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/bench/box/box_385_t.obj"});


  std::cout << "Body COG position : ("
            << body->GetCOGPositionInWorld(NWU).GetX() << ","
            << body->GetCOGPositionInWorld(NWU).GetY() << ","
            << body->GetCOGPositionInWorld(NWU).GetZ() << ")"
            << std::endl;

  auto code = solve_hydrostatic_equilibrium(body, boxMesh, FrFrame());


  std::cout<<"Body orientation : "<< body->GetRotation()<<std::endl;
}