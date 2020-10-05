//
// Created by camille on 30/09/2020.
//

#include "frydom/frydom.h"

using namespace frydom;

enum CASES {
  STATIC,
  HARMONIC_SURGE,
  HARMONIC_SWAY,
  HARMONIC_HEAVE,
  AIRY_0_DEG,
  AIRY_90_DEG
};

enum LINK_CASES {
  MOTOR,
  SET_MOTION
};

CASES BENCH_CASE = HARMONIC_SURGE;
LINK_CASES BENCH_LINK = SET_MOTION;

void InitializeEnvironment(FrOffshoreSystem &system, double wave_height, double wave_period, double wave_dir) {

  // Seabed
  auto seabed = system.GetEnvironment()->GetOcean()->GetSeabed();
  seabed->Show(true);
  seabed->SetBathymetry(-200, NWU);
  seabed->GetSeabedGridAsset()->SetGrid(-1000, 1000, 200, -1000, 1000, 200);

  // FreeSurface
  auto ocean = system.GetEnvironment()->GetOcean();
  ocean->ShowFreeSurface(true);
  ocean->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(-1000, 1000, 200, -1000, 1000, 200);
  ocean->GetFreeSurface()->GetFreeSurfaceGridAsset()->UpdateAssetON();

  // Current
  ocean->GetCurrent()->MakeFieldUniform();
  ocean->GetCurrent()->GetFieldUniform()->Set(90, 0., DEG, KNOT, NED, GOTO);

  // Density
  ocean->SetDensity(1025.);

  // Gravity
  system.SetGravityAcceleration(9.81);

  // Waves
  auto waveField = ocean->GetFreeSurface()->SetAiryRegularWaveField(wave_height, wave_period, wave_dir,
      DEG, NWU, GOTO);

  waveField->SetStretching(WHEELER);

  ocean->GetFreeSurface()->GetFreeSurfaceGridAsset()->UpdateAssetON();

  system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0., 0., wave_period, 1.);
  system.GetEnvironment()->GetTimeRamp()->SetActive(true);

}

std::shared_ptr<FrCableProperties> InitializeCableProperties() {

  double diameter = 0.0766; // m
  double axial_stiffness = 7.536E8; // N
  double linear_density = 113.35; // kg/m

  double section = 0.25 * M_PI * diameter * diameter; // m^2
  double young_modulus = axial_stiffness / section;
  double density = linear_density / section; // kg/m^3

  auto cable_properties = make_cable_properties();
  cable_properties->SetLinearDensity(linear_density);
  cable_properties->SetDiameter(diameter);
  cable_properties->SetYoungModulus(young_modulus);
  cable_properties->SetDragCoefficients(1.1, 0.);
  cable_properties->SetAddedMassCoefficients(1., 0.);

  return cable_properties;
}

std::shared_ptr<FrPrismaticLink> make_motor_link(FrOffshoreSystem& system,
    std::shared_ptr<FrNode> node_1, std::shared_ptr<FrNode> node_2,
    double period, double motion_amplitude) {

  auto link = make_prismatic_link("link", &system, node_1, node_2);

  auto t = new_var();
  double w = MU_2PI / period;
  auto ramp = FrCosRampFunction();
  ramp.SetByTwoPoints(0., period, 0., 1.);
  auto motor_function = motion_amplitude * cos(w * t);

  auto motor = link->Motorize("motor", POSITION);
  motor->SetMotorFunction(motor_function);

  return link;
}

int main() {

  FrOffshoreSystem system("bench_FEACable_FOWT_moored");

  double cable_length = 835.500; // m
  int nb_elements = 167;

  // Anchor points
  auto world_body = system.GetWorldBody();

  auto G1 = world_body->NewNode("G1");
  G1->SetPositionInWorld({418.8, 725.383, -200.}, NWU);

  auto G2 = world_body->NewNode("G2");
  G2->SetPositionInWorld({-837.6, 0., -200.}, NWU);

  auto G3 = world_body->NewNode("G3");
  G3->SetPositionInWorld({418.8, -725.383, -200.}, NWU);

  // Support body
  auto body = system.NewBody("body");
  body->SetPosition({0., 0., 0.}, NWU);

  auto body_asset = FrFileSystem::join({system.config_file().GetDataFolder(),
                                        "ce/bench/FOWT/Base_01.obj"});
  body->AddMeshAsset(body_asset);
  body->SetColor(Yellow);

  // Fair-lead points

  auto T1 = body->NewNode("T1");
  T1->SetPositionInBody({20.435, 35.394458, -14}, NWU);

  auto T2 = body->NewNode("T2");
  T2->SetPositionInBody({-40.87, 0., -14}, NWU);

  auto T3 = body->NewNode("T3");
  T3->SetPositionInBody({20.435, -35.394458, -14}, NWU);

  auto body_node = body->NewNode("body_node");
  auto body_fixed_node = world_body->NewNode("body_fixed_node");
  body_fixed_node->SetPositionInWorld({0., 0., 0.}, NWU);

  // Motion

  double motion_amplitude = 0.;
  double period = 40.;
  double wave_height = 0.;
  double wave_period = 10.;
  double wave_dir = 0.;

  switch (BENCH_CASE) {
    case STATIC:
      break;
    case HARMONIC_SURGE:
      motion_amplitude = 10.;
      body_fixed_node->RotateAroundYInWorld(MU_PI_2, NWU);
      body_node->RotateAroundYInWorld(MU_PI_2, NWU);
      break;
    case HARMONIC_SWAY:
      motion_amplitude = 10.;
      body_fixed_node->RotateAroundXInWorld(MU_PI_2, NWU);
      body_node->RotateAroundXInWorld(MU_PI_2, NWU);
      break;
    case AIRY_0_DEG:
      wave_height = 5.;
      wave_period = 27.;
      wave_dir = 0.;
      break;
    case AIRY_90_DEG:
      wave_height = 5.;
      wave_period = 27.;
      wave_dir = 90.;
      break;
  }
  InitializeEnvironment(system, wave_height, wave_period, wave_dir);

  // Create cable properties
  auto cable_properties = InitializeCableProperties();

  // Create the cables
  auto cable_1 = make_fea_cable("cable_1", G1, T1, cable_properties, cable_length, nb_elements);
  auto cable_2 = make_fea_cable("cable_2", G2, T2, cable_properties, cable_length, nb_elements);
  auto cable_3 = make_fea_cable("cable_3", G3, T3, cable_properties, cable_length, nb_elements);

  // Set Motion

  std::shared_ptr<FrLink> link;

  auto ramp = FrCosRampFunction();
  ramp.SetByTwoPoints(40., 0., 40+period, motion_amplitude);
  auto t = new_var();
  auto function = ramp * sin(MU_2PI/period * t);
  function.SetXOffset(0.);
  function.Initialize();

  switch (BENCH_LINK) {
    case MOTOR:
      link = make_motor_link(system, body_node, body_fixed_node, period, motion_amplitude);
      break;
    case SET_MOTION:
      body->SetFixedInWorld(true);
      break;
  }

  // Solver
  system.SetSolver(FrOffshoreSystem::SOLVER::MINRES);
  system.SetSolverWarmStarting(true);
  system.SetSolverMaxIterSpeed(1000);
  system.SetSolverMaxIterStab(1000);
  system.SetSolverForceTolerance(1e-7);
  system.SetSolverDiagonalPreconditioning(true);

  // Time step
  double dt = 0.01;
  double t_max = 160.;

  system.SetTimeStep(dt);

  // Initialize system
  system.Initialize();

  // Solve static
  system.GetStaticAnalysis()->SetNbIteration(40);
  system.GetStaticAnalysis()->SetNbSteps(50);
  system.SolveStaticWithRelaxation();

  // Run simulation
  bool is_irrlicht = true;

  if (is_irrlicht) {
    system.RunInViewer(t_max, 100., false, 10);
  } else {
    double time = 0.;
    while(time < t_max) {

      if (motion_amplitude > DBL_EPSILON and BENCH_LINK == SET_MOTION) {
        body->SetPosition({function.Get_y(time), 0., 0.}, NWU);
        body->SetVelocityInWorldNoRotation({function.Get_y_dx(time), 0., 0.}, NWU);
        body->SetAccelerationInWorldNoRotation({function.Get_y_dxdx(time), 0., 0.}, NWU);
      }

      system.AdvanceTo(time);
      std::cout << "time = " << time << " s ; ";
      std::cout << "surge : " << function.Get_y(time) << std::endl;
      time += dt;
    }
  }

  return 0;
}
