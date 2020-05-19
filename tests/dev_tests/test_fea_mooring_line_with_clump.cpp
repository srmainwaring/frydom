//
// Created by frongere on 14/05/2020.
//

#include "frydom/frydom.h"

//#define TARGET_ELEMENT_LENGTH 10


using namespace frydom;

void InitializeEnvironment(FrOffshoreSystem &system) {
  auto seabed = system.GetEnvironment()->GetOcean()->GetSeabed();
  seabed->Show(true);
  // FIXME: le no show seabed ne doit pas declencher de profondeur infine !!! Ca doit seulement concerner l'asset !!
  seabed->SetBathymetry(-100, NWU); // TODO: depth target -100m
  seabed->GetSeabedGridAsset()->SetGrid(-520, 520, 1040, -520, 520, 1040);

  system.GetEnvironment()->GetOcean()->ShowFreeSurface(true);
  system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(
      -100, 100, 3, -100, 100, 200);

  system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
  system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform()->Set(
      90, 3.,
      DEG, KNOT, NED, GOTO);

  double Hs = 5.;
  double Tp = 10.;
  double wave_dir = 0.;
  system.GetEnvironment()->GetOcean()->GetFreeSurface()->SetAiryRegularOptimWaveField(Hs, Tp, wave_dir, DEG, NWU, GOTO);

  system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->UpdateAssetON();

  system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0., 0., 3., 1.);
  system.GetEnvironment()->GetTimeRamp()->SetActive(true);

}

std::shared_ptr<FrCableProperties> InitializeCableProperties() {
  double diam_ = 0.168; // m
  double EA_ = 602.68e6; // N
  double lambda_ = 141; // kg/m


  double A_ = M_PI * diam_ * diam_ / 4.; // m**2
  double E = EA_ / A_; // Pa
  double density = lambda_ / A_; // kg/m**3

  auto cable_properties = make_cable_properties();
  cable_properties->SetLinearDensity(lambda_);
  cable_properties->SetDiameter(diam_);
  cable_properties->SetYoungModulus(E);
  cable_properties->SetDragCoefficients(1.2, 0.);
  cable_properties->SetAddedMassCoefficients(2, 0.);
  cable_properties->SetHydrodynamicDiameter(0.168);
  cable_properties->SetRayleighDamping(1e4);
  cable_properties->SetVIVAmpFactor(3.54);

  return cable_properties;
}

std::shared_ptr<FrFEACable> MakeLineWithClump(FrOffshoreSystem *system,
                                              double short_cable_length,
                                              double long_cable_length,
                                              std::shared_ptr<FrCableProperties> cable_properties,
                                              Position refPosition,
                                              double heading,
                                              double anchor_distance,
                                              double clump_weight_distance_from_triplate,
                                              int lineNumber) {

  auto short_cable_nb_elements = 5;
  auto long_cable_nb_elements = 30;

  auto suffix = std::to_string(lineNumber);

  auto anchor = system->GetEnvironment()->GetOcean()->GetSeabed()->NewAnchor("anchor" + suffix,
                                                                             refPosition,
                                                                             heading,
                                                                             anchor_distance,
                                                                             DEG,
                                                                             NED);


  auto triplate = system->NewBody("triplate" + suffix);
  triplate->SetPosition(refPosition, heading, 64., DEG, NED);
  makeItSphere(triplate, 0.4, 60);
  auto triplate_node = triplate->NewNode("triplate" + suffix + "_node");

  auto clump_weight = system->NewBody("clump_weight" + suffix);
  clump_weight->SetPosition(triplate->GetPosition(NWU) + Position{0., 0., -clump_weight_distance_from_triplate}, NWU);
  makeItSphere(clump_weight, 0.5, 120e3);
  auto clump_weight_node = clump_weight->NewNode("clump_weight" + suffix + "_node");
  clump_weight_node->SetPositionInBody({0., 0., clump_weight_distance_from_triplate}, NWU);

  make_spherical_link("clump_weight" + suffix + "_link", system, clump_weight_node, triplate_node);


  auto fairlead = system->GetWorldBody()->NewNode("fairlead" + suffix);
  fairlead->SetPositionInWorld(refPosition, NWU);



  // Creating the cable
  auto long_cable = make_fea_cable("long_cable" + suffix,
                                   anchor,
                                   triplate_node,
                                   cable_properties,
                                   long_cable_length,
                                   long_cable_nb_elements);

  // Creating the cable
  auto short_cable = make_fea_cable("short_cable" + suffix,
                                    triplate_node,
                                    fairlead,
                                    cable_properties,
                                    short_cable_length,
                                    short_cable_nb_elements);


}

int main() {

  FrOffshoreSystem system("test_fea_mooring_line_with_clump");
  InitializeEnvironment(system);


  double short_cable_length = 64; // m
  double long_cable_length = 519 - 64; // m


  // Create cable properties
  auto cable_properties = InitializeCableProperties();

  Position refPosition = {0., 0., 0.};

  auto line1 = MakeLineWithClump(&system,
                                 short_cable_length,
                                 long_cable_length,
                                 cable_properties,
                                 refPosition,
                                 330.,
                                 500.076,
                                 5.,
                                 1);
  auto line2 = MakeLineWithClump(&system,
                                 short_cable_length,
                                 long_cable_length,
                                 cable_properties,
                                 refPosition,
                                 30.,
                                 500.072,
                                 5.,
                                 2);
  auto line3 = MakeLineWithClump(&system,
                                 short_cable_length,
                                 long_cable_length,
                                 cable_properties,
                                 refPosition,
                                 90.,
                                 502.815,
                                 5.,
                                 3);
  auto line4 = MakeLineWithClump(&system,
                                 short_cable_length,
                                 long_cable_length,
                                 cable_properties,
                                 refPosition,
                                 150.,
                                 504.477,
                                 5.,
                                 4);
  auto line5 = MakeLineWithClump(&system,
                                 short_cable_length,
                                 long_cable_length,
                                 cable_properties,
                                 refPosition,
                                 210.,
                                 504.477,
                                 5.,
                                 5);
  auto line6 = MakeLineWithClump(&system,
                                 short_cable_length,
                                 long_cable_length,
                                 cable_properties,
                                 refPosition,
                                 270.,
                                 502.815,
                                 5.,
                                 6);



  // TODO: faire ces settings en auto en cas d'utilisation de cable fea... (ou voir si ce n'est pas un reglage standard
  // pour toute simulation

  system.SetSolver(FrOffshoreSystem::SOLVER::MINRES);
  system.SetSolverWarmStarting(true);
  system.SetSolverMaxIterSpeed(500);
  system.SetSolverMaxIterStab(500);
  system.SetSolverForceTolerance(1e-14);
  system.SetSolverDiagonalPreconditioning(true);

  system.SetTimeStep(0.01);


//  auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
//  msolver->SetVerbose(false);
//  msolver->SetDiagonalPreconditioning(true);
//  application.SetTimestep(0.01);

  system.RunInViewer(0., 100., true, 10);

  return 0;
}
