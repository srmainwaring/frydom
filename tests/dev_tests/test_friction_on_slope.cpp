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

#include "chrono/physics//ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;

//#define HAS_IRRLICHT

#ifdef HAS_IRRLICHT
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;
#endif

int slope(double deg) {

  // Print to file
  std::ofstream myfile;
  myfile.open("slope_" + std::to_string(deg) + ".csv", std::ios::out);

  myfile << "time;z;\ns;m;" << std::endl;

  ChSystemNSC system;

  auto floor = std::make_shared<ChBodyEasyBox>(50,1,50,100,
                                               true, true, ChMaterialSurface::ContactMethod::NSC);
  floor->SetBodyFixed(true);
  auto floorColor = std::make_shared<ChColorAsset>(0.7,0.1,0.5,0.5);
  floor->AddAsset(floorColor);
  chrono::ChQuaternion<double> floorRotation;
  double angle = deg*M_PI/180;
  floorRotation.Q_from_AngX(angle);
  floor->SetRot(floorRotation);

  auto box = std::make_shared<ChBodyEasyBox>(1,1,1,1000,
                                             true, true, ChMaterialSurface::ContactMethod::NSC);
  auto boxColor = std::make_shared<ChColorAsset>(0.1,0.7,0.5,0.5);
  box->AddAsset(boxColor);
  box->SetRot(floorRotation);
  box->SetPos(ChVector<double>(0.,cos(angle),sin(angle)));

  system.AddBody(floor);
  system.AddBody(box);

  // Modify some setting of the physical system for the simulation, if you want
  system.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
  system.SetMaxItersSolverSpeed(100);
  system.SetMaxItersSolverStab(100);

  #ifdef HAS_IRRLICHT
  // Create the Irrlicht visualization
  ChIrrApp application(&system, L"friction on a box", core::dimension2d<u32>(800, 600), false);

  // Add camera, lights, logo and sky in Irrlicht scene
//  application.AddTypicalLogo();
  application.AddTypicalSky();
  application.AddTypicalLights();
  application.AddTypicalCamera(core::vector3df(0, 14, 20));

  // Complete asset specification: convert all assets to Irrlicht
  application.AssetBindAll();
  application.AssetUpdateAll();

  // Simulation loop
  application.SetTimestep(0.01);

  while (application.GetDevice()->run()) {
    application.BeginScene();
    application.DrawAll();
    application.DoStep();
    myfile << system.GetChTime() << ";" << box->GetPos().z() << ";" << std::endl;
    application.EndScene();
  }
  #else
  double dt = 0.01;
  double time = -dt;
  while(time<100) {
    time += dt;
    system.DoFrameDynamics(time);
    myfile << system.GetChTime() << ";" << box->GetPos().z() << ";" << std::endl;
    std::cout<<"time : "<<time<<std::endl;
//    std::cout<<"iter : "<<system.GetSolver()->GetIterLog()<<std::endl;
  }
  #endif


  myfile.close();


}

int main() {

//  for (auto deg : {10,20,30,40}) {
//    slope(deg);
//    std::cout << "deg "<< deg << " done"<< std::endl;
//  }

  slope(32.);

}