//
// Created by camille on 29/09/2020.
//

#include <frydom/frydom.h>

using namespace frydom;

enum CASES {
  STATIC,
  HARMONIC_SURGE,
  HARMONIC_SWAY,
  HARMONIC_HEAVE,
  AIRY_0_DEG,
  AIRY_45_DEG,
  AIRY_90_DEG
};

CASES BENCH_CASE = HARMONIC_SURGE;

void InitializeEnvironment(FrOffshoreSystem &system, double wave_height, double wave_period, double wave_dir) {

  // Seabed

  auto seabed = system.GetEnvironment()->GetOcean()->GetSeabed();
  seabed->Show(true);
  seabed->SetBathymetry(-100, NWU);
  seabed->GetSeabedGridAsset()->SetGrid(-150, 50, 200, -50, 50, 100);

// Free surface
  system.GetEnvironment()->GetOcean()->ShowFreeSurface(true);
  system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(
      -150, 50, 2, -50, 50, 100);
  system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->UpdateAssetON();

  // Current
  system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
  system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform()->Set(
      90, 0.,
      DEG, KNOT, NED, GOTO);

  // water props
  system.GetEnvironment()->GetOcean()->SetDensity(1000.);

  // Gravity
  system.SetGravityAcceleration(9.807);


  auto waveField = system.GetEnvironment()->GetOcean()->GetFreeSurface()->SetAiryRegularWaveField(wave_height, wave_period, wave_dir,
                                                                                                  DEG, NWU, GOTO);

  waveField->SetStretching(WHEELER);

  system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->UpdateAssetON();


  system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0., 0., 3., 1.);
  system.GetEnvironment()->GetTimeRamp()->SetActive(true);


}

std::shared_ptr<FrCableProperties> InitializeCableProperties() {

  double diameter = 0.396; // m
  double axial_stiffness = 5e8; // N
  double linear_density = 165.; // kg/m

  double section = 0.25 * M_PI * diameter * diameter; // m**2
  double young_modulus = axial_stiffness / section;
  double density = linear_density / section; // kg/m**3

  auto cable_properties = make_cable_properties();
  cable_properties->SetLinearDensity(linear_density);
  cable_properties->SetDiameter(diameter);
  cable_properties->SetYoungModulus(young_modulus);
  cable_properties->SetDragCoefficients(1., 0.);
  cable_properties->SetAddedMassCoefficients(1., 0.);

  return cable_properties;

}

int main() {

  FrOffshoreSystem system("bench_fea_cable_mooring");

  double cable_length = 170.;
  int nb_elements = 68;

  // Create anchor
  auto world_body = system.GetWorldBody();
  auto anchor = world_body->NewNode("anchor");
  anchor->SetPositionInWorld({-120, 0., -100.}, NWU);

  // Creating a body to support the right node
  auto support_body = system.NewBody("support_body");
  support_body->SetPosition({0., 0., -5}, NWU);

  auto body_node = support_body->NewNode("body_node");

  auto body_fixed_node = world_body->NewNode("body_fixed_node");
  body_fixed_node->SetPositionInWorld({0., 0., -5.}, NWU);

  double motion_amplitude = 0.;
  double period = 50;
  double wave_height = 0.;
  double wave_period = 10.;
  double wave_dir = 0.;

  switch (BENCH_CASE) {

    case STATIC:
      break;

    case HARMONIC_SURGE:
      motion_amplitude = 5.;
      body_fixed_node->RotateAroundYInWorld(MU_PI_2, NWU);
      body_node->RotateAroundYInWorld(MU_PI_2, NWU);
      break;

    case HARMONIC_SWAY:
      motion_amplitude = 10.;
      body_fixed_node->RotateAroundXInWorld(M_PI_2, NWU);
      body_node->RotateAroundXInWorld(MU_PI_2, NWU);
      break;

    case AIRY_0_DEG:
      wave_height = 5.;
      break;

    case AIRY_45_DEG:
      wave_height = 5.;
      wave_dir = 45.;
      break;

    case AIRY_90_DEG:
      wave_height = 5.;
      wave_dir = 90.;
      break;

  }

  InitializeEnvironment(system, wave_height, wave_period, wave_dir);

  // Create cable properties
  auto cable_properties = InitializeCableProperties();

  // Creating the cable
  auto cable = make_fea_cable("cable", anchor, body_node, cable_properties, cable_length, nb_elements);

  // Motor link
  auto link = make_prismatic_link("link", &system, body_node, body_fixed_node);

  auto t = new_var();
  double w = MU_2PI / period;
  auto motor_function = motion_amplitude * cos(w * t);

  auto motor = link->Motorize("motor", POSITION);
  motor->SetMotorFunction(motor_function);

  // Solver
  system.SetSolver(FrOffshoreSystem::SOLVER::MINRES);
  system.SetSolverMaxIterations(1000);
  system.SetSolverForceTolerance(1e-7);
  //system.SetSolverDiagonalPreconditioning(true);

  system.SetTimeStep(0.01);

  system.Initialize();

  system.GetStaticAnalysis()->SetNbIteration(50);
  system.GetStaticAnalysis()->SetNbSteps(20);
  system.SolveStaticWithRelaxation();

  system.RunInViewer(100., 100., false, 10);

  return 0;

}