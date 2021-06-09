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

  auto manFile = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx027/sutulo_manoeuvring_model.json"});
  auto man = make_Sutulo_manoeuvring_model("manModel", body.get(), manFile);

  auto propFile = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx027/propeller_FPP.json"});
  auto prop = make_first_quadrant_propeller_force("prop", body.get(), Position(4.83, 0., 1.), propFile, NWU);
  prop->SetDiameter(2.);
  prop->SetScrewDirection(frydom::FrPropellerForce::RIGHT_HANDED);
  prop->SetRotationalVelocity(100, RPM);
  prop->SetThrustDeductionFactor(0.);
  prop->SetStraightRunWakeFraction(0.);
  prop->SetCorrectionFactor(1.);

  system.SetTimeStep(0.01);

  system.RunInViewer();

}
