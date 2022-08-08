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

#include "frydom/frydom.h"

#include "chrono/physics//ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace frydom;

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

TEST(friction, chrono) {
  bool verbose = true;

  double friction = 0.6;

  auto target_coulomb_angle = atan(friction) * RAD2DEG;
  std::cout << "static friction coefficient : " << friction << std::endl;
  std::cout << "target Coulomb angle : " << target_coulomb_angle << std::endl;

  ChSystemNSC system;

  auto mat = ChMaterialSurface::DefaultMaterial(ChContactMethod::NSC);
  mat->SetFriction(friction);

  auto floor = std::make_shared<ChBodyEasyBox>(50, 1, 50, 100, true, true, mat);
  floor->SetBodyFixed(true);

  auto box = std::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, true, true, mat);
  box->SetPos(ChVector<double>(0., 1., 0.));

  auto constantForce = std::make_shared<ChForce>();
  box->AddForce(constantForce);
  constantForce->SetVrelpoint(ChVector<double>());
  auto function = std::make_shared<ChFunction_Const>(-box->GetMass() * system.Get_G_acc().y());
  constantForce->SetF_x(function);

  system.AddBody(floor);
  system.AddBody(box);

  // Modify some setting of the physical system for the simulation, if you want
  system.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
  system.SetSolverMaxIterations(100);

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

TEST(friction, static_friction) {
  bool verbose = false;

  FrOffshoreSystem system("test_friction", FrOffshoreSystem::NONSMOOTH_CONTACT,
                   FrOffshoreSystem::EULER_IMPLICIT_LINEARIZED, FrOffshoreSystem::APGD);

  auto floor = system.NewBody("floor");
  floor->SetFixedInWorld(true);
  auto mat = FrMaterialSurfaceNSC();
  mat.static_friction = 0.6;
  makeItBox(floor, 10, 10, 1, 1E3, &mat);

  auto box = system.NewBody("box");
  makeItBox(box, 1, 1, 1, 1E3, &mat);
  box->SetPosition(Position(0.,0.,1), NWU);

  Force tangent(box->GetMass() * system.GetGravityAcceleration(),0. ,0.);
  auto tangentForce = make_constant_force("tangent", box->NewNode("center"), frydom::FrConstantForce::FOLLOWING,
                                          Force(box->GetMass() * system.GetGravityAcceleration(),0. ,0.), NWU);

  system.AdvanceOneStep(0.01);

  auto contactForce = box->GetContactForceInWorld(NWU);
  double ratio = abs(contactForce.GetFx()/contactForce.GetFz());

  auto relative_error = abs(ratio - mat.static_friction) / mat.static_friction;
  if (verbose) {
    std::cout << "ratio : " << ratio << std::endl;
    std::cout << "relative error : " << relative_error << std::endl;
  }
  EXPECT_TRUE(relative_error < 1E-12);
}