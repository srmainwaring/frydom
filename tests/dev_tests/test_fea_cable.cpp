//
// Created by frongere on 29/04/2020.
//

#include "frydom/frydom.h"


using namespace frydom;

void InitializeEnvironment(FrOffshoreSystem &system) {
  auto seabed = system.GetEnvironment()->GetOcean()->GetSeabed();
  seabed->Show(true);
  // FIXME: le no show seabed ne doit pas declencher de profondeur infine !!! Ca doit seulement concerner l'asset !!
  seabed->SetBathymetry(-30, NWU); // TODO: depth target -100m
  seabed->GetSeabedGridAsset()->SetGrid(-500, 500, 500, -50, 50, 50);

  system.GetEnvironment()->GetOcean()->ShowFreeSurface(true);
  system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(
      -500, 500, 500, -50, 50, 50);

  system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
  system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform()->Set(
      45, 1.,
      DEG, KNOT, NED, GOTO);
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

  return cable_properties;
}

int main() {

  FrOffshoreSystem system("test_fea_cable");
  InitializeEnvironment(system);

  // Create nodes
  auto world_body = system.GetWorldBody();

  auto start_node = world_body->NewNode("start_node");
  auto end_node = world_body->NewNode("end_node");

  Position start_node_pos = {0., 0., 0.};
  Position end_node_pos = {500., 0., 0.};

  start_node->SetPositionInWorld(start_node_pos, NWU);
  end_node->SetPositionInWorld(end_node_pos, NWU);

  // Create cable properties
  auto cable_properties = InitializeCableProperties();

  unsigned int cable_nb_elements = 200;

  double cable_length = (end_node_pos - start_node_pos).norm(); // m

  // Creating the cable
  auto cable = make_fea_cable("cable",
                              start_node,
                              end_node,
                              cable_properties,
                              cable_length,
                              cable_nb_elements);

//  cable->SetEndLinkType(FrFEACable::FREE); // TODO: simplifier l'API en ayant des methodes adaptees
//  cable->SetStartLinkType(FrFEACable::FREE); // TODO: simplifier l'API en ayant des methodes adaptees



  // TODO: le faire en auto !!

  system.SetSolver(FrOffshoreSystem::SOLVER::MINRES);
  system.SetSolverWarmStarting(true);
  system.SetSolverMaxIterSpeed(500);
  system.SetSolverMaxIterStab(500);
  system.SetSolverForceTolerance(1e-14);

  system.SetTimeStep(0.01);


//  auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
//  msolver->SetVerbose(false);
//  msolver->SetDiagonalPreconditioning(true);
//
//  application.SetTimestep(0.01);



  system.RunInViewer();

  return 0;
}
