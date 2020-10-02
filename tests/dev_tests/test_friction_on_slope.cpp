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

#define DEG2RAD (M_PI/180)
#define RAD2DEG (180/M_PI)

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

bool slope(double deg, double friction = 0.5, bool verbose = false, bool output = false) {

  // Print to file
  std::ofstream myfile;
  if (output) {
    myfile.open("slope_" + std::to_string(deg) + ".csv", std::ios::out);
    myfile << "time;z;\ns;m;" << std::endl;
  }

  ChSystemNSC system;

  auto floor = std::make_shared<ChBodyEasyBox>(50, 1, 50, 100,
                                               true, true, ChMaterialSurface::ContactMethod::NSC);
  floor->SetBodyFixed(true);
  auto floorColor = std::make_shared<ChColorAsset>(0.7, 0.1, 0.5, 0.5);
  floor->AddAsset(floorColor);
  floor->GetMaterialSurfaceNSC()->SetSfriction(1.);

  auto box = std::make_shared<ChBodyEasyBox>(1, 1, 1, 1000,
                                             true, true, ChMaterialSurface::ContactMethod::NSC);
  auto boxColor = std::make_shared<ChColorAsset>(0.1, 0.7, 0.5, 0.5);
  box->AddAsset(boxColor);
  box->SetPos(ChVector<double>(0., 1., 0.));
  box->GetMaterialSurfaceNSC()->SetSfriction(friction);

  auto mg = -box->GetMass() * system.Get_G_acc().y();
  auto tangentialForce = mg * tan(deg * M_PI / 180);
  if (verbose) {
    std::cout << "angle : " << deg << "°" << std::endl;
    std::cout << "Mass x g : " << mg << "kg" << std::endl;
    std::cout << "Tangential Force : " << tangentialForce << "N" << std::endl;
  }

  auto constantForce = std::make_shared<ChForce>();
  box->AddForce(constantForce);
  constantForce->SetVrelpoint(ChVector<double>());
  auto function = std::make_shared<ChFunction_Const>(tangentialForce);
  constantForce->SetF_x(function);

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
    if (output) myfile << system.GetChTime() << ";" << box->GetPos().z() << ";" << std::endl;
    application.EndScene();
  }
  #else
  double dt = 0.01;
  double time = -dt;
  while(time<100) {
    time += dt;
    system.DoFrameDynamics(time);
    if (output)
      myfile << system.GetChTime() << ";" << box->GetPos().z() << ";" << std::endl;
    if (verbose) {
//      std::cout<<"time : "<<time<<std::endl;
//      std::cout<<"iter : "<<system.GetSolver()->GetIterLog()<<std::endl;
    }
  }
  #endif

  if (output)
    myfile.close();

  if (verbose)
    std::cout << "box position : " << box->GetPos().x() << std::endl;

  return box->GetPos().x() < 1E-5;
}

double dichotomy(double friction, bool verbose = true) {

  auto target_angle = atan(friction) * RAD2DEG;

  double eps = 1e-8;
  double a = target_angle-1.;
  double b = target_angle+1.;
  double m;

  bool m_ok;
  while (abs(b-a)>eps) {
    m = 0.5*(a+b);
    m_ok = slope(m, friction, true);
    if (verbose)
      std::cout<<"angle "<< m <<" : "<< m_ok << std::endl;
    if (m_ok) {
      a = m;
    }
    else {
      b = m;
    }
  }

  if (m_ok) {
    std::cout << " Coulomb angle = " << m << std::endl;
    return m;
  } else {
    std::cout<<" Coulomb angle = " << a << std::endl;
    return a;
  }


}



int main() {

//  slope(30.9637, 0.6, true, true);

  auto friction = 0.6;
  auto target_angle = atan(friction) * RAD2DEG;
  std::cout << "target Coulomb angle = " << target_angle << std::endl;

  auto coulomb_angle = dichotomy(friction);
  auto friction_coeff = tan(coulomb_angle*DEG2RAD);
  std::cout << "friction coeff = " << friction_coeff << std::endl;

  auto relative_error = abs(friction - friction_coeff)/friction;
  std::cout << "relative error = " << 100*relative_error << std::endl;

  return relative_error > 1E-5;

}