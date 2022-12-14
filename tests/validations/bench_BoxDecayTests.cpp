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

  FrOffshoreSystem system("bench_NonlinearHydrostaticBox");

  auto Ocean = system.GetEnvironment()->GetOcean();
  Ocean->SetDensity(1023.);

  // -- Body
  auto body = system.NewBody("Box");

  double L, B, H, c;
  L = 8;
  B = 4;
  H = 2;
  c = 0.5;

  auto mass = L * H * B * c * system.GetEnvironment()->GetFluidDensity(WATER);
  makeItBox(body, L, B, H, mass);

//    body->RemoveAssets();

  bool linear = false;
  auto boxMesh = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/bench/box/box_250.obj"});
  if (linear) {
    // -- Linear hydrostatics
    auto eqFrame = make_equilibrium_frame("EqFrame", body);

    auto forceHst = make_linear_hydrostatic_force("linear_hydrostatic", body, eqFrame, boxMesh, FrFrame());
  } else {
    // Nonlinear hydrostatics
    auto bodyMesh = make_hydro_mesh("boxMesh", body, boxMesh, FrFrame(), FrHydroMesh::ClippingSupport::PLANESURFACE);
//        bodyMesh->ShowAsset(true);
    //bodyMesh->GetInitialMesh().Write("Mesh_Initial.obj");
    auto forceHst = make_nonlinear_hydrostatic_force("nonlinear_hydrostatic", body, bodyMesh);
  }


  auto dampingForce = make_linear_damping_force("linear_damping", body, WATER, true);
  dampingForce->SetDiagonalDamping(1E4, 1E4, 1E4, 1E5, 1E5, 1E5);

  // -- Simulation

  auto dt = 0.01;

  system.SetTimeStep(dt);
  system.Initialize();

  // Decay test initial position.
  FrRotation decayRot;
  decayRot.SetCardanAngles_DEGREES(20., 0., 0., NWU);

//  body->SetPosition(Position(0., 0., 2.), NWU);
  body->SetRotation(decayRot);

  bool irrlicht = true;

  if (irrlicht) {
    system.RunInViewer(250., 10, true, 20);
  } else {

    auto time = 0.;
    clock_t begin = clock();

    while (time < 300.) {
      time += dt;

      system.AdvanceTo(time);
      std::cout << "time : " << time << std::endl;
    }

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Elapsed cpu time in seconds : " << elapsed_secs << std::endl;
    std::cout << "============================== End ===================================== " << std::endl;
  }

}
