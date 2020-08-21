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
               " Benchmark test : Recursive convolution \n"
               " ===================================================== " << std::endl;

  // -- System

  bool recursive_radiation = true;
  std::string body_type = "barge";

  std::string simulation_name;

  if (recursive_radiation) {
    simulation_name = "Recursive_convolution";
  } else {
    simulation_name = "Classic_convolution";
  }
  simulation_name += body_type;

  FrOffshoreSystem system(simulation_name);

  auto Ocean = system.GetEnvironment()->GetOcean();
  Ocean->SetDensity(1000);

  // -- Body

  auto body = system.NewBody(body_type);

  Position COGPosition(0., 0., 0.);

  // -- Inertia

  std::string HDB_file;
  if (body_type == "sphere") {
    body->SetInertiaTensor(FrInertiaTensor(2094., 837, 837, 837, 0., 0., 0., COGPosition, NWU));
    body->GetDOFMask()->SetLock_X(true);
    body->GetDOFMask()->SetLock_Y(true);
    body->GetDOFMask()->SetLock_Rx(true);
    body->GetDOFMask()->SetLock_Ry(true);
    body->GetDOFMask()->SetLock_Rz(true);
    HDB_file = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/bench/hemisphere/Hemisphere.hdb5"});
  } else if (body_type == "barge") {
    body->SetInertiaTensor(FrInertiaTensor(5.125E7, 2.05E10, 2.05E10, 2.05E10, 0., 0., 0., COGPosition, NWU));
    HDB_file = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/bench/boxbarge/BoxBarge.hdb5"});
  }

  // -- Hydrodynamics
  auto hdb = make_hydrodynamic_database(HDB_file);

  auto eqFrame = make_equilibrium_frame("EqFrame", body);

  hdb->Map(0, body.get(), eqFrame);

  // -- Linear hydrostatics

  auto forceHst = make_linear_hydrostatic_force("linear_hydrostatic", body, hdb);

  // -- Radiation

  if (recursive_radiation) {
    auto radiationModel = make_recursive_convolution_model("radiation_convolution", &system, hdb);
  } else {
    auto radiationModel = make_radiation_convolution_model("radiation_convolution", &system, hdb);
//    radiationModel->SetImpulseResponseSize(body.get(), 6., 0.01);
  }

  // motor
//  auto sphereNode = body->NewNode("sphere_node");
//  sphereNode->RotateAroundYInBody(90*DEG2RAD, NWU);
//  auto WBNode = system.GetWorldBody()->NewNode("WB_node");
//  WBNode->RotateAroundYInBody(90*DEG2RAD, NWU);
//
//  auto link = make_prismatic_link("link", &system, WBNode, sphereNode);
//  auto motor = link->Motorize("motor", ACTUATOR_CONTROL::POSITION);
//  auto x = new_var("x");
//  motor->SetMotorFunction(sin(2*MU_PI/10 * x));

  // -- Simulation

  auto dt = 0.01;

  system.SetTimeStep(dt);
  system.Initialize();

  // Decay test initial position.
  body->SetPosition(Position(0., 0., 1.), NWU);
  if (body_type == "barge") {
    body->RotateAroundCOG(FrRotation(Direction(0., 1., 0.0), 2 * DEG2RAD, NWU), NWU);
  }

  auto time = 0.;

  clock_t begin = clock();

  while (time < 100.) {
    time += dt;
    system.AdvanceTo(time);
    std::cout << "time : " << time << std::endl;
  }

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "Elapsed cpu time in seconds : " << elapsed_secs << std::endl;
  std::cout << "============================== End ===================================== " << std::endl;

} // end namespace frydom
