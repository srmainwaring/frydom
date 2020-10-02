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

#include "gtest/gtest.h"
#include "chrono/physics//ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;

#define RAD2DEG (180/M_PI)
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

TEST(friction, coulomb_angle) {
  bool verbose = false;

  double friction = 0.5;

  auto target_coulomb_angle = atan(friction) * RAD2DEG;
  std::cout << "static friction coefficient : " << friction << std::endl;
  std::cout << "target Coulomb angle : " << target_coulomb_angle << std::endl;

  ChSystemNSC system;

  auto floor = std::make_shared<ChBodyEasyBox>(50, 1, 50, 100,
                                               true, true, ChMaterialSurface::ContactMethod::NSC);
  floor->SetBodyFixed(true);
  floor->GetMaterialSurfaceNSC()->SetSfriction(1.);

  auto box = std::make_shared<ChBodyEasyBox>(1, 1, 1, 1000,
                                             true, true, ChMaterialSurface::ContactMethod::NSC);
  box->SetPos(ChVector<double>(0., 1., 0.));
  box->GetMaterialSurfaceNSC()->SetSfriction(friction);

  auto constantForce = std::make_shared<ChForce>();
  box->AddForce(constantForce);
  constantForce->SetVrelpoint(ChVector<double>());
  auto function = std::make_shared<ChFunction_Const>(-box->GetMass() * system.Get_G_acc().y());
  constantForce->SetF_x(function);

  system.AddBody(floor);
  system.AddBody(box);

  // Modify some setting of the physical system for the simulation, if you want
  system.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
  system.SetMaxItersSolverSpeed(100);
  system.SetMaxItersSolverStab(100);

  system.DoStepDynamics(0.01);

  auto contact_force = system.GetContactContainer()->GetContactableForce(box.get());
  auto ratio = abs(contact_force.x()/contact_force.y());
//  auto coulomb_angle = atan(ratio) * RAD2DEG;

  auto relative_error = abs(ratio - friction)/friction;
//  auto relative_error_bis = abs(target_coulomb_angle - coulomb_angle)/target_coulomb_angle;

  if (verbose) {
    std::cout << "ratio : " << ratio << std::endl;
//    std::cout << "Coulomb angle : " << coulomb_angle << std::endl;

    std::cout << "relative error : " << relative_error << std::endl;
//    std::cout << "relative error bis : " << relative_error_bis << std::endl;
  }
  EXPECT_TRUE(relative_error < 1E-6);
}