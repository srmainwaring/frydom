//
// Created by frongere on 07/06/2020.
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


CASES BENCH_CASE = STATIC;


void InitializeEnvironment(FrOffshoreSystem &system, double wave_height, double wave_period, double wave_dir) {

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


  double cable_length = 170.; // m
//  int nb_elements = 68;

  int bspline_order = 2;
  int nb_elements = 15; // 44

  double dt = 0.01;
  double t_max = 100;

  bool is_irrlicht = false;

  // Create left node (always fixed)
  auto world_body = system.GetWorldBody();
  auto left_node = world_body->NewNode("left_node");
  left_node->SetPositionInWorld({-100., 0., -55.}, NWU);

  // Creating a body to support the right node
  auto support_body = system.NewBody("support_body");
  support_body->SetPosition({0., 0., -5.}, NWU);

  auto right_body = system.NewBody("body");
//  body->SetFixedInWorld(true);
  right_body->SetPosition({0., 0., -5.}, NWU);

  auto right_node = right_body->NewNode("right_node");
////  right_node->RotateAroundYInWorld(MU_PI_2, NWU); // Case 1
//  right_node->RotateAroundXInWorld(MU_PI_2, NWU); // Case 2 -> FIXME la liaison echoue



  auto right_fixed_node = world_body->NewNode("right_fixed_node");
  right_fixed_node->SetPositionInWorld({0., 0., -5.}, NWU);
////  right_fixed_node->RotateAroundYInWorld(MU_PI_2, NWU);  // Case 1
//  right_fixed_node->RotateAroundXInWorld(MU_PI_2, NWU);  // Case 2 -> FIXME la liaison echoue


  // FIXME: dans le cas d'une traversee de surface libre, il convient d'appliquer le taux d'immersion tel que le fait
  // Orcaflex sous le nom "proportion wet". Ca devrait permettre d'eviter les discontinuites dans la tension...
  // https://www.orcina.com/webhelp/OrcaFlex/Content/html/Linetheory,Interactionwiththeseasurface.htm
  // Ou alors, on detecte un segment qui traverse la surface libre et on lui applique une quadrature de gauss a plus de
  // points...




  // TODO: ajouter des methodes pour les ayant un axe (prismatiq et revolute notamment) qui permettent de placer l'axe
  // comme on veut en une passe...


  // TODO: le comportement bruite de la tension dans la ligne peut venir de 2 choses:
  // 1- des pb de convergence du solveur de contrainte
  // 2- une reponse haute frequence due à l'emploi des elements finis tel que decrit dans la doc Orcaflex a
  // numerical damping
  // 3-




//  // Defining the morotr function
//  double motion_amplitude = 0.;
//  double period = 27.;




  double motion_amplitude = 0.;
  double period = 27.;
  double wave_height = 0.;
  double wave_period = 10.;
  double wave_dir = 0.;

  switch (BENCH_CASE) {

    case STATIC:
      break;

    case HARMONIC_SURGE:
      motion_amplitude = 10.;
      right_fixed_node->RotateAroundYInWorld(MU_PI_2, NWU);
      right_node->RotateAroundYInWorld(MU_PI_2, NWU);
      break;

    case HARMONIC_SWAY: // FIXME: ici la liaison ne tient pas...
      motion_amplitude = 10.;
      right_fixed_node->RotateAroundXInWorld(MU_PI_2, NWU);
      right_node->RotateAroundXInWorld(MU_PI_2, NWU);
      break;

    case HARMONIC_HEAVE:
      motion_amplitude = 10.;
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
  auto cable = make_fea_cable("cable",
                              left_node,
                              right_node,
                              cable_properties,
                              cable_length,
                              nb_elements,
                              bspline_order);


  auto link = make_prismatic_link("link", &system, right_node, right_fixed_node);

  auto t = new_var();
  double w = MU_2PI / period;
  auto motor_function = motion_amplitude * cos(w * t);

  auto motor = link->Motorize("motor", POSITION);
  motor->SetMotorFunction(motor_function);

//  system.GetEnvironment()->GetOcean()->GetFreeSurface()->SetAiryRegularWaveField(waveHeight, wavePeriod, waveDirAngle,
//                                                                                 DEG, NWU, GOTO);


  system.SetSolver(FrOffshoreSystem::SOLVER::MINRES);
  system.SetSolverMaxIterations(1000);
  system.SetSolverForceTolerance(1e-9);
  system.SetSolverDiagonalPreconditioning(true);

  system.SetTimeStep(dt);

  if (is_irrlicht) {
    system.RunInViewer(t_max, 100., false, 10);
  } else {
    double time = 0;
    while (time < t_max) {
      time += dt;
      system.AdvanceTo(time);
      std::cout << "time = " << time << " s" << std::endl;
    }
  }




//  double A = 10.;
//  double period = 27.;
//
//
//  double time = 0.;
//  double dt = 1e-2;
//
//  while (time < 80.) {
//
//    double x = A * std::sin(MU_2PI * time / period);
//    double xp = A * (MU_2PI / period) * std::cos(MU_2PI * time / period);
//
//    right_body->SetPosition({x, 0., -5.}, NWU);
//    right_body->SetVelocityInWorldNoRotation({xp, 0., 0.}, NWU);
//
////    std::cout << time << ": " << x << std::endl;
//
//    system.AdvanceOneStep(dt);
//
////    std::cout << body->GetCOGPositionInWorld(NWU)[0] << std::endl;
//
//    time += dt;
//  }

  return 0;
}

