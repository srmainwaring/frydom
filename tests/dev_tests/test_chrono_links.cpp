//
// Created by lletourn on 18/11/2020.
//


//#include "chrono/core/ChRealtimeStep.h"
//#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
//#include "chrono_irrlicht/ChIrrApp.h"


// Use the namespaces of Chrono
//using namespace chrono;
//using namespace chrono::irrlicht;
//
// Use the main namespaces of Irrlicht
//using namespace irr;
//using namespace irr::core;
//using namespace irr::scene;
//using namespace irr::video;
//using namespace irr::io;
//using namespace irr::gui;

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {

  FrOffshoreSystem system("test_chrono_links");

  auto body1 = system.NewBody("body1");
  makeItBox(body1, 10, 1, 1, 10000);
  body1->AllowCollision(false);

  auto sliding1 = std::make_shared<chrono::ChLinkMotorLinearSpeed>();
  sliding1->Initialize(internal::GetChronoBody(body1), internal::GetChronoBody(system.GetWorldBody()), chrono::ChFrame<>(chrono::ChVector<>(0, 0, 0)));
  sliding1->SetName("sliding1");
  internal::GetChronoSystem(&system)->AddLink(sliding1);
//  auto my_speed_function = std::make_shared<chrono::ChFunction_Sine>(0, 0.1, 1);
  auto my_speed_function = std::make_shared<chrono::ChFunction_Const>(1);
  sliding1->SetSpeedFunction(my_speed_function);


  auto body2 = system.NewBody("body2");
  makeItBox(body2, 1, 5, 1, 100);
  body2->AllowCollision(false);

//  auto prismatic2 = std::make_shared<chrono::ChLinkLockPrismatic>();
//  prismatic2->Initialize(internal::GetChronoBody(body2), internal::GetChronoBody(body1), chrono::ChCoordsys<>(chrono::ChVector<>(0, 0, 0), -MU_PI_2, chrono::ChVector<>(1,0,0)));
//  prismatic2->SetName("prismatic2");
//  internal::GetChronoSystem(&system)->AddLink(prismatic2);

  auto sliding2 = std::make_shared<chrono::ChLinkMotorLinearSpeed>();
  sliding2->Initialize(internal::GetChronoBody(body2), internal::GetChronoBody(body1), chrono::ChFrame<>(chrono::ChVector<>(0, 0, 0), MU_PI_2, chrono::ChVector<>(0,0,1)));
  sliding2->SetName("sliding2");
//  sliding2->SetGuideConstraint(chrono::ChLinkMotorLinear::GuideConstraint::FREE);
  internal::GetChronoSystem(&system)->AddLink(sliding2);
  auto my_speed_function2 = std::make_shared<chrono::ChFunction_Sine>(0, 0.1, 1);
  sliding2->SetSpeedFunction(my_speed_function2);

//  system.SetSolverGeometricTolerance(1E-6);
  system.SetSolverMaxIterSpeed(100);
//  system.SetSolverMaxIterAssembly(100);
//  system.SetSolverMaxIterStab(100);

  system.RunInViewer();

}
//
//int main2(int argc, char* argv[]) {
//  GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
//
//  //
//  // HERE YOU CREATE THE MECHANICAL SYSTEM OF CHRONO...
//  //
//
//  // 1- Create a ChronoENGINE physical system: all bodies and constraints
//  //    will be handled by this ChSystemNSC object.
//
//  ChSystemNSC my_system;
//
//  // 2- Create the rigid bodies of the slider-crank mechanical system
//  //   (a crank, a rod, a truss), maybe setting position/mass/inertias of
//  //   their center of mass (COG) etc.
//
//  // Ground
//  auto ground = std::make_shared<ChBody>();
//  my_system.AddBody(ground);
//  ground->SetBodyFixed(true);
//  ground->SetName("Ground");
//
//  // first sliding body
//  auto body1 = std::make_shared<ChBody>();
//  my_system.AddBody(body1);
//  body1->SetMass(2);
//  body1->SetName("Body1");
//
//  // second sliding body, wrt body1
//  auto body2 = std::make_shared<ChBody>();
//  my_system.AddBody(body2);
//  body2->SetMass(3);
//  body2->SetName("Rod");
//
//  auto sliding1 = std::make_shared<ChLinkMotorLinearSpeed>();
//  sliding1->Initialize(ground, body1, ChFrame<>(ChVector<>(0, 0, 0)));
//  sliding1->SetName("sliding1");
//  my_system.AddLink(sliding1);
//  auto my_speed_function = std::make_shared<ChFunction_Const>(1);
//  sliding1->SetSpeedFunction(my_speed_function);
//
//  auto sliding2 = std::make_shared<ChLinkMotorLinearSpeed>();
//  sliding2->Initialize(body1, body2, ChFrame<>(ChVector<>(0, 0, 0)));
//  sliding2->SetName("sliding2");
//  my_system.AddLink(sliding2);
//  auto my_speed_function2 = std::make_shared<ChFunction_Sine>(0, 0.1, 1);
//  sliding2->SetSpeedFunction(my_speed_function2);
//
//
//  // 4- Create the Irrlicht visualization
//  ChIrrApp application(&my_system, L"Simple slider-crank example", core::dimension2d<u32>(800, 600), false);
//
//  // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
//  ChIrrWizard::add_typical_Logo(application.GetDevice());
//  ChIrrWizard::add_typical_Sky(application.GetDevice());
//  ChIrrWizard::add_typical_Lights(application.GetDevice());
//  ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 0, -10));
//
//  // Bind assets
//  application.AssetBindAll();
//  application.AssetUpdateAll();
//
//  //
//  // THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
//  //
//
//  // This will help choosing an integration step which matches the
//  // real-time step of the simulation..
//  ChRealtimeStepTimer m_realtime_timer;
//
//  while (application.GetDevice()->run()) {
//    // Irrlicht must prepare frame to draw
//    application.BeginScene(true, true, SColor(255, 140, 161, 192));
//
//    // Irrlicht now draws simple lines in 3D world representing a
//    // skeleton of the mechanism, in this instant:
//    //
//    // .. draw items belonging to Irrlicht scene, if any
//    application.DrawAll();
//    // .. draw a grid
//    ChIrrTools::drawGrid(application.GetVideoDriver(), 0.5, 0.5);
//    // .. draw GUI items belonging to Irrlicht screen, if any
//    application.GetIGUIEnvironment()->drawAll();
//
//    // HERE CHRONO INTEGRATION IS PERFORMED: THE
//    // TIME OF THE SIMULATION ADVANCES FOR A SINGLE
//    // STEP:
//
//    my_system.DoStepDynamics(m_realtime_timer.SuggestSimulationStep(0.02));
//
//    // Irrlicht must finish drawing the frame
//    application.EndScene();
//  }
//
//  return 0;
//}
