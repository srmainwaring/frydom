//
// Created by frongere on 07/06/2020.
//

#include <frydom/frydom.h>

using namespace frydom;


void InitializeEnvironment(FrOffshoreSystem &system) {

  // Seabed
  auto seabed = system.GetEnvironment()->GetOcean()->GetSeabed();
  seabed->Show(true);
  // FIXME: le no show seabed ne doit pas declencher de profondeur infine !!! Ca doit seulement concerner l'asset !!
  seabed->SetBathymetry(-100, NWU); // TODO: depth target -100m
  seabed->GetSeabedGridAsset()->SetGrid(-150, 50, 200, -50, 50, 100);

  // Free surface
  system.GetEnvironment()->GetOcean()->ShowFreeSurface(true);
  system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(
      -150, 50, 2, -50, 50, 100);

  // Current
  system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
  system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform()->Set(
      90, 0.,
      DEG, KNOT, NED, GOTO);

  // water props
  system.GetEnvironment()->GetOcean()->SetDensity(1000.);

  // Gravity
  system.SetGravityAcceleration(9.807);

//  // Waves
//  double Hs = 0.;
//  double Tp = 10.;
//  double wave_dir = 0.;
//  system.GetEnvironment()->GetOcean()->GetFreeSurface()->SetAiryRegularOptimWaveField(Hs, Tp, wave_dir, RAD, NWU, GOTO);
  double waveHeight = 5.;
  double wavePeriod = 10.;
  double waveDirAngle = 0.;

  system.GetEnvironment()->GetOcean()->GetFreeSurface()->SetAiryRegularWaveField(waveHeight, wavePeriod, waveDirAngle,
      DEG, NWU, GOTO);

  system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->UpdateAssetON();


  system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0., 0., 3., 1.);
  system.GetEnvironment()->GetTimeRamp()->SetActive(true);

}

std::shared_ptr<FrCableProperties> InitializeCableProperties() {
  double diam_ = 0.396; // m
  double EA_ = 5e8; // N
  double lambda_ = 165; // kg/m


  double A_ = M_PI * diam_ * diam_ / 4.; // m**2
  double E = EA_ / A_; // Pa
  double density = lambda_ / A_; // kg/m**3

  auto cable_properties = make_cable_properties();
  cable_properties->SetLinearDensity(lambda_);
  cable_properties->SetDiameter(diam_);
  cable_properties->SetYoungModulus(E);
  cable_properties->SetDragCoefficients(1., 0.);
  cable_properties->SetAddedMassCoefficients(1., 0.);
//  cable_properties->SetHydrodynamicDiameter(0.168);
//  cable_properties->SetRayleighDamping(1e4);
//  cable_properties->SetVIVAmpFactor(3.54);

  return cable_properties;
}

int main() {

  std::cout << "Bench of FEA cable feature with respect to Orcaflex bench" << std::endl
            << "https://www.orcina.com/wp-content/uploads/resources/validation/99-102-Low-and-Langley-OMAE-2006.pdf"
            << std::endl;


  FrOffshoreSystem system("test_fea_cable");
  InitializeEnvironment(system);

  // Create nodes
  auto world_body = system.GetWorldBody();


  double cable_length = 170.; // m
//  int nb_elements = 68;
  int nb_elements = 68; // 44


  auto left_node = world_body->NewNode("left_node");
  auto right_node = world_body->NewNode("right_node");

  left_node->SetPositionInWorld({-100., 0., -55.}, NWU);
  right_node->SetPositionInWorld({0., 0., -5.}, NWU);

  // Create cable properties
  auto cable_properties = InitializeCableProperties();

  // Creating the cable
  auto cable = make_fea_cable("cable",
                              left_node,
                              right_node,
                              cable_properties,
                              cable_length,
                              nb_elements);

  system.SetSolver(FrOffshoreSystem::SOLVER::MINRES);
  system.SetSolverWarmStarting(true);
  system.SetSolverMaxIterSpeed(500);
  system.SetSolverMaxIterStab(500);
  system.SetSolverForceTolerance(1e-14);
  system.SetSolverDiagonalPreconditioning(true);

//  system.SetTimeStepper(frydom::FrOffshoreSystem::NEWMARK);

  system.SetTimeStep(0.01);

  system.RunInViewer(0., 100., false, 10);


//double time = 0.;
//  system.RunDynamics(1e-2);


  return 0;
}

