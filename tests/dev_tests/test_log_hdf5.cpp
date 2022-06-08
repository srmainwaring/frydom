//
// Created by frongere on 16/04/2021.
//

#include "frydom/frydom.h"

using namespace frydom;

int main() {

  FrOffshoreSystem system("Essai");

  auto body = system.NewBody("body");
  makeItSphere(body, 2, 50);

  system.Initialize();

  auto log_mgr = system.GetLogManager();

  log_mgr->LogCSV(false);
  log_mgr->LogHDF5(true);



  double dt = 1e-1;
  double time = 0.;

  do {
    system.AdvanceOneStep(dt);
    time += dt;
  } while (time < 10);

  return 0;
}