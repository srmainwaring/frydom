//
// Created by camille on 21/07/2020.
//

#include "frydom/frydom.h"

using namespace frydom;

enum element_case {ec_X, ec_Y, ec_Z, ec_XY, ec_YZ, ec_XYZ};

std::shared_ptr<FrMorisonCompositeElement> make_morison_model(
    const std::string& name, std::shared_ptr<FrBody> body, bool is_extended, element_case type) {

  auto morisonModel = make_morison_model(name, body, is_extended);

  switch (type) {
    case ec_X:
      morisonModel->AddElement({-2., 0., -10.}, {2., 0., -10.}, 1., 0.8, 0.6, 0.);
      break;
    case ec_Y:
      morisonModel->AddElement({0., -2., -10.}, {0., 2., -10.}, 1., 0.8, 0.6, 0.);
      break;
    case ec_Z:
      morisonModel->AddElement({0., 0., -12.}, {0., 0., -8}, 1., 0.8, 0.6, 0.);
      break;
    case ec_XY:
      morisonModel->AddElement({-1.4142, -1.4142, -10.}, {1.4142, 1.4142, -10.}, 1., {0.8, 0.8, 0.}, 0.6, 0.);
      break;
    case ec_YZ:
      morisonModel->AddElement({0., -1.4142, -11.4142}, {0., 1.4142, -8.5858}, 1., 0.8, 0.6, 0.);
      break;
    case ec_XYZ:
      morisonModel->AddElement({-1.4142, -1.4142, -11.4142}, {1.4142, 1.4142, -8.5858}, 1., 0.8, 0.6, 0.);
      break;
  }
  return morisonModel;
}

int main(int argc, char* argv[]) {

  // System

  FrOffshoreSystem system("bench_MorisonSingImmersedElement");

  // Environment

  double wavePeriod = 8.;
  double waveAmplitude = 0.1;

  auto ocean = system.GetEnvironment()->GetOcean();
  auto waveField = ocean->GetFreeSurface()->SetAiryRegularWaveField();
  waveField->SetWavePeriod(wavePeriod);
  waveField->SetWaveHeight(waveAmplitude);

  system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0., 0., wavePeriod, 1.);
  system.GetEnvironment()->GetTimeRamp()->SetActive(true);

  // Body

  auto body = system.NewBody("body");
  body->SetPosition({0., 0., 0.}, NWU);
  body->GetDOFMask()->MakeItLocked();

  // Morison model

  auto morisonModel = make_morison_model("morison", body, true, ec_XY);
  auto morisonForce = make_morison_force("morison", body, morisonModel);

  // Simulation

  double dt = 0.01;
  double t_end = 10 * wavePeriod;
  double time = 0.;

  system.SetTimeStep(dt);
  system.Initialize();

  while (time < t_end) {
    system.AdvanceTo(time);
    std::cout << "time = " << time << " s " << std::endl;
    time += dt;
  }

}