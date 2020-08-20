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

  std::string simulation_name;

  if (recursive_radiation) {
    simulation_name = "Recursive_convolution";
  } else {
    simulation_name = "Classic_convolution";
  }

  FrOffshoreSystem system(simulation_name);

  auto Ocean = system.GetEnvironment()->GetOcean();
  Ocean->SetDensity(1000);

  // -- Body

  auto body = system.NewBody("sphere");

  Position COGPosition(0., 0., 0.);

  body->SetPosition(Position(0., 0., 0.), NWU);

  // -- Inertia

  double mass = 2094.;

  double Ixx = 837;
  double Iyy = 837;
  double Izz = 837;

  FrInertiaTensor InertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., COGPosition, NWU);

  body->SetInertiaTensor(InertiaTensor);

  body->GetDOFMask()->SetLock_X(true);
  body->GetDOFMask()->SetLock_Y(true);
  body->GetDOFMask()->SetLock_Rx(true);
  body->GetDOFMask()->SetLock_Ry(true);
  body->GetDOFMask()->SetLock_Rz(true);

  // -- Hydrodynamics

  //auto hdb = make_hydrodynamic_database(resources_path.resolve("sphere_hdb.h5").path());
  auto sphere_HDB = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/bench/hemisphere/test.hdb5"});
  auto hdb = make_hydrodynamic_database(sphere_HDB);

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
  // -- Simulation

  auto dt = 0.01;

  system.SetTimeStep(dt);
  system.Initialize();

  // Decay test initial position.
  body->SetPosition(Position(0., 0., 1.), NWU);

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
