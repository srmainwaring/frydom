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

int main(int argc, char* argv[]) {

  FrOffshoreSystem system("test_FrSteadyStateChecker",
      FrOffshoreSystem::SYSTEM_TYPE::SMOOTH_CONTACT,
      FrOffshoreSystem::TIME_STEPPER::EULER_IMPLICIT_LINEARIZED,
      FrOffshoreSystem::SOLVER::MINRES);

  // -- body

  auto body = system.NewBody("ship");

  auto body_node = body->NewNode("body_node");
  auto body_fixed_node = system.GetWorldBody()->NewNode("body_fixed_node");
  body_fixed_node->SetPositionInWorld({0., 0., 0.}, NWU);
  body_node->RotateAroundYInWorld(MU_PI_2, NWU);
  body_fixed_node->RotateAroundYInWorld(MU_PI_2, NWU);

  // -- Link

  auto link = make_prismatic_link("link", &system, body_node, body_fixed_node);

  auto t = new_var();
  auto ramp = FrCosRampFunction();
  ramp.SetByTwoPoints(0., 0, 30., 1.);

  auto motor_function = 0.1 * ramp * t;

  auto motor = link->Motorize("motor", POSITION);
  motor->SetMotorFunction(motor_function);

  // -- Steady State Checker

  auto checker = FrSteadyStateChecker(20, 0.1, 10.);
  checker.AddField<double>([body]() { return body->GetVelocityInBody(NWU).GetVx(); }, 0.001);
  checker.AddField<double>([body]() { return body->GetVelocityInBody(NWU).GetVy(); }, 0.001);

  // -- Simulation

  double dt = 0.01;
  double t_max = 100.;

  system.SetTimeStep(dt);

  // -- Initialize system
  system.Initialize();

  double time = 0.;
  while (time < t_max and not checker.IsConverged()) {

    time += dt;
    system.AdvanceTo(time);
    std::cout << "time = " << time << " seconds" << std::endl;

    checker.Record(time); // TODO : a passer dans FrOffshoreSystem ?!

  }
  return 0;
}