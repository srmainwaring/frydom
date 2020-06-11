//
// Created by frongere on 29/04/2020.
//

#include "frydom/frydom.h"


using namespace frydom;

void InitializeEnvironment(FrOffshoreSystem &system) {
  auto seabed = system.GetEnvironment()->GetOcean()->GetSeabed();
  seabed->Show(true);
  // FIXME: le no show seabed ne doit pas declencher de profondeur infine !!! Ca doit seulement concerner l'asset !!
  seabed->SetBathymetry(-100, NWU); // TODO: depth target -100m
  seabed->GetSeabedGridAsset()->SetGrid(-20, 500, 520, -50, 50, 100);

  system.GetEnvironment()->GetOcean()->ShowFreeSurface(true);
  system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(
      -20, 500, 2, -50, 50, 100);

  system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
  system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform()->Set(
      90, 0.,
      DEG, KNOT, NED, GOTO);

  double Hs = 5.;
  double Tp = 10.;
  double wave_dir = 0.;
  system.GetEnvironment()->GetOcean()->GetFreeSurface()->SetAiryRegularOptimWaveField(Hs, Tp, wave_dir, RAD, NWU, GOTO);

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

int main() {

  FrOffshoreSystem system("test_fea_cable");
  InitializeEnvironment(system);

  // Create nodes
  auto world_body = system.GetWorldBody();


  double cable_length = 519; // m

  Position start_node_pos_base = {500., 0., -100.};
  Position end_node_pos_base = {0., 0., 0.};

  std::vector<std::pair<double, unsigned int>> conf = {
//      {-30, 20},
//      {-20, 30},
//      {-10, 50},
      {0,   75},
//      {10,  100},
//      {20,  125},
//      {30,  150},
  };

  std::shared_ptr<FrFEACable> cable;

  for (const auto &pair : conf) {

    auto start_node = world_body->NewNode("start_node_" + std::to_string(pair.second));
    auto end_node = world_body->NewNode("end_node_" + std::to_string(pair.second));
    // FIXME: si on ne met pas l'ancre en premier dans le cable, l'initialistion du shape se passe mal...

    auto start_node_pos = start_node_pos_base;
    start_node_pos.y() += pair.first;

    auto end_node_pos = end_node_pos_base;
    end_node_pos.y() += pair.first;

    start_node->SetPositionInWorld(start_node_pos, NWU);
    end_node->SetPositionInWorld(end_node_pos, NWU);

    // Create cable properties
    auto cable_properties = InitializeCableProperties();

    unsigned int cable_nb_elements = pair.second;

    // Creating the cable
    cable = make_fea_cable("cable_" + std::to_string(pair.second),
                           start_node,
                           end_node,
                           cable_properties,
                           cable_length,
                           cable_nb_elements);

    //  cable->SetEndLinkType(FrFEACable::FREE); // TODO: simplifier l'API en ayant des methodes adaptees
//      cable->SetStartLinkType(FrFEACable::FREE); // TODO: simplifier l'API en ayant des methodes adaptees
  }


  // Adding a clump weight
  auto clump_weight = cable->AddClumpWeight("clump_weight", cable_length - 64., 1);
  clump_weight->SetSubmergedMass(120e3);
//  clump_weight->SetDryMass(120e3);
  clump_weight->SetAsCylinder(1., 1.);
  clump_weight->SetMorisonCoefficients(0.52, 0.91, 1.595, 1.637);


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
