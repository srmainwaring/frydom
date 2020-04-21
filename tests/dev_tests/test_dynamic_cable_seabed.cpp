//
// Created by frongere on 10/04/2020.
//

#include <chrono/fea/ChElementBase.h>
#include "frydom/frydom.h"


using namespace frydom;

int main() {

  FrOffshoreSystem system("test_lumped_mass_cable");

  auto seabed = system.GetEnvironment()->GetOcean()->GetSeabed();
  seabed->Show(true);
  // FIXME: le no show seabed ne doit pas declencher de profondeur infine !!! Ca doit seulement concerner l'asset !!
  seabed->SetBathymetry(-100, NWU);
  seabed->GetSeabedGridAsset()->SetGrid(-500, 500, 500, -50, 50, 50);

  system.GetEnvironment()->GetOcean()->ShowFreeSurface(true);
  system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(
      -500, 500, 500, -50, 50,50);

  system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
  system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform()->Set(90, 0., DEG, KNOT, NED, GOTO);

  auto world_body = system.GetWorldBody();


//  auto anchor = seabed->NewAnchor("anchor", -500., 0., NWU);
  auto anchor = world_body->NewNode("anchor");
  anchor->SetPositionInWorld({-500, 0, -100}, NWU);


  auto sphere = system.NewBody("sphere");
  makeItSphere(sphere, 1, 1000);
  sphere->SetPosition({0., 0., 0.}, NWU);
  sphere->SetFixedInWorld(true);


  auto cylinder_anchor = sphere->NewNode("cylinder_anchor");

  double cable_length = 64 + 425 + 30; // Adding more length...
  // 5 m / element est ce qui est fait par defaut dans ARIANE3D apparemment
//  int nb_elements = int(cable_length / 5); // TODO: voir dans quelle mesure on peut augmenter la discretisation
  int nb_elements = 10; // TODO: voir dans quelle mesure on peut augmenter la discretisation


  auto cable_properties = make_cable_properties();

//  // DONNEES A-FLOWT
  cable_properties->SetLinearDensity(
      141); // FIXME: submerged weight = 122... Comment le prendre en compte comme cela ??? --> trouver un diametre equivalent...
  cable_properties->SetDiameter(0.168); // Vrai valeur
  //  cable_properties->SetEA(602.59e6); // Vrai valeur // FIXME: EA est systematiquement calcule alors que c'est ca qu'on veut...
  cable_properties->SetYoungModulus(602.59e6 / cable_properties->GetSectionArea());
  cable_properties->SetDragCoefficients(1.2, 0.);
  cable_properties->SetAddedMassCoefficients(2, 0.);
  cable_properties->SetHydrodynamicDiameter(0.168);
  cable_properties->SetRayleighDamping(1e4);


  auto cable = make_dynamic_cable("cable",
                                  anchor,
                                  cylinder_anchor,
                                  cable_properties,
                                  cable_length,
                                  cable_properties->GetRayleighDamping(), // TODO: mettre dans
                                  nb_elements);

//  // Essai de cast...
//  auto element_IGA = std::make_shared<internal::FrElementBeamIGA>();
//  std::shared_ptr<chrono::fea::ChElementBase> element_base = element_IGA;
//  auto element_back = std::dynamic_pointer_cast<internal::FrElementBeamIGA>(element_base);


//  system.SetTimeStepper(FrOffshoreSystem::TIME_STEPPER::RUNGEKUTTA45);

  system.SetSolver(FrOffshoreSystem::SOLVER::MINRES);
  system.SetSolverWarmStarting(true);
  system.SetSolverMaxIterSpeed(500);
  system.SetSolverMaxIterStab(500);
  system.SetSolverForceTolerance(1e-14);

  system.Initialize();

  system.SetTimeStep(1e-2);
  system.RunInViewer(0., 200,true);

  return 0;
}
