//
// Created by camille on 22/07/2020.
//

#include "frydom/frydom.h"
#include "frydom/hydrodynamics/buoyancy/FrBarElement.h"
#include "frydom/hydrodynamics/buoyancy/FrBuoyancyBarElements.h"

using namespace frydom;

enum element_case {ec_X, ec_Y, ec_Z, ec_XY, ec_YZ, ec_XYZ};

std::shared_ptr<FrMorisonCompositeElement> make_morison_model(
    const std::string& name, std::shared_ptr<FrBody> body, bool is_extended, element_case type) {

  auto morisonModel = make_morison_model(name, body, is_extended);

  switch (type) {
    case ec_X:
      morisonModel->AddElement({-2., 0., 0.}, {2., 0., 0.}, 1., 0.8, 0.6, 0., 20);
      break;
    case ec_Y:
      morisonModel->AddElement({0., -2., 0.}, {0., 2., 0.}, 1., 0.8, 0.6, 0., 20);
      break;
    case ec_Z:
      morisonModel->AddElement({0., 0., -2.}, {0., 0., 2.}, 1., 0.8, 0.6, 0., 20);
      break;
    case ec_XY:
      morisonModel->AddElement({-1.4142, -1.4142, 0.}, {1.4142, 1.4142, -0.}, 1., 0.8, 0.6, 0., 20);
      break;
    case ec_YZ:
      morisonModel->AddElement({0., -1.4142, -1.4142}, {0., 1.4142, 1.4142}, 1., 0.8, 0.6, 0., 20);
      break;
    case ec_XYZ:
      morisonModel->AddElement({-1.4142, -1.4142, -1.4142}, {1.4142, 1.4142, 1.4142}, 1., 0.8, 0.6, 0., 20);
      break;
  }
  return morisonModel;
}

std::shared_ptr<FrCompositeBarElement> make_bar_element(std::shared_ptr<FrBody> body, element_case type) {

  auto barModel = make_bar_element(body);

  switch (type) {
    case ec_X:
      barModel->AddElement({-2., 0., 0.}, {2., 0., 0.}, 0.5);
      break;
    case ec_Y:
      barModel->AddElement({0., -2., 0.}, {0., 2., 0.}, 0.5);
      break;
    case ec_Z:
      barModel->AddElement({0., 0., -2.}, {0., 0., 2.}, 0.5);
      break;
    case ec_XY:
      barModel->AddElement({-1.4142, -1.4142, 0.}, {1.4142, 1.4142, -0.}, 0.5);
      break;
    case ec_YZ:
      barModel->AddElement({0., -1.4142, -1.4142}, {0., 1.4142, 1.4142}, 0.5);
      break;
    case ec_XYZ:
      barModel->AddElement({-1.4142, -1.4142, -1.4142}, {1.4142, 1.4142, 1.4142}, 0.5);
      break;
  }
  return barModel;
}


int main(int argc, char* argv[]) {

  element_case ec = ec_YZ;

  // System

  FrOffshoreSystem system("bench_MorisonCrossingElement");

  // Environment

  double wavePeriod = 8.;
  double waveAmplitude = 0.1;

  auto ocean = system.GetEnvironment()->GetOcean();
  auto waveField = ocean->GetFreeSurface()->SetAiryRegularWaveField();
  waveField->SetWavePeriod(wavePeriod);
  waveField->SetWaveHeight(waveAmplitude);
  waveField->SetStretching(WHEELER);

  system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0., 0., wavePeriod, 1.);
  system.GetEnvironment()->GetTimeRamp()->SetActive(true);

  // Body

  auto body = system.NewBody("body");
  body->SetPosition({0., 0., 0.}, NWU);
  body->GetDOFMask()->MakeItLocked();

  // Morison model

  auto morisonModel = make_morison_model("morison", body, true, ec);
  auto morisonForce = make_morison_force("morison", body, morisonModel);

  // Bar Element Model

  auto barModel = make_bar_element(body, ec);
  auto barForce = make_buoyancy_bar_elements("buoyancy", body, barModel);

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