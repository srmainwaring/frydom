//
// Created by lletourn on 09/06/2021.
//

#include "frydom/frydom.h"

using namespace frydom;

int main() {

  FrOffshoreSystem system("test_rdx027");

  auto body = system.NewBody("vessel");

  body->SetInertiaTensor(FrInertiaTensor(345000, 6.2E6, 1.35E7, 1.35E7, 0., 0., 0., Position(10.945, 0., 3.5), NWU));
  // draft
  body->TranslateInWorld(0., 0., -3.114, NWU);

  body->AddMeshAsset(FrFileSystem::join({system.config_file().GetDataFolder(), "rdx027/TPS_for_hydrostatics.obj"}));

  body->GetDOFMask()->LockXYPlane();

  auto eqFrame = make_spring_damping_equilibrium_frame("eqFrame", body, 100., 0.8);

  auto hdb = make_hydrodynamic_database(FrFileSystem::join({system.config_file().GetDataFolder(), "rdx027/TPS_sym_VF.hdb5"}));
  hdb->Map(0, body.get(), eqFrame);

  auto radiationModel = make_recursive_convolution_model("radiation", &system, hdb);

  auto manFile = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx027/sutulo_maneuvering_model.json"});
//  auto man = make_Sutulo_manoeuvring_model("manModel", body.get(), manFile);

  auto propFile = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx027/propeller_FPP.json"});
  auto prop = make_first_quadrant_propeller_force("prop", body.get(), Position(4.83, 0., 1.), propFile, NWU);
  prop->SetDiameter(2.);
  prop->SetScrewDirection(frydom::FrPropellerForce::RIGHT_HANDED);
  prop->SetRotationalVelocity(100, RPM);
  prop->SetThrustDeductionFactor(0.);
  prop->SetStraightRunWakeFraction(0.);
  prop->SetCorrectionFactor(1.);

  auto rudderNode = body->NewNode("rudderNode");
  rudderNode->SetPositionInBody(Position(0., 0., 0.83), NWU);

//  auto rudderFile = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx027/rudder.json"});
//  auto rudder = make_rudder_force("rudder", body.get(), rudderNode, rudderFile);
//  rudder->SetStraightRunWakeFraction(0.);
//  rudder->SetHeight(1.68);
//  rudder->SetProjectedLateralArea(1.848);
//  rudder->SetRudderAngle(0.*DEG2RAD);

  body->AddBoxShape(1.1, 0.14, 1.68, rudderNode->GetNodePositionInBody(NWU), NWU);

  system.SetTimeStep(0.01);

//  system.RunInViewer();

  system.AdvanceTo(6000);

}
