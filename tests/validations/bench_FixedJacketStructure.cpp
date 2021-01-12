//
// Created by camille on 09/07/2020.
//

#include "frydom/frydom.h"
#include "frydom/hydrodynamics/buoyancy/FrBarElement.h"
#include "frydom/hydrodynamics/buoyancy/FrBuoyancyBarElements.h"

using namespace frydom;

int main(int argc, char *argv[]) {

  std::cout << " =================================================================== \n"
               " Benchmark : Fixed Jacket Structure \n"
               " =================================================================== \n" << std::endl;

  // System

  FrOffshoreSystem system("bench_FixedJacketStructure");
  system.SetGravityAcceleration(9.807);

  // Environment

  auto ocean = system.GetEnvironment()->GetOcean();
  ocean->SetDensity(1025.);

  double waveHeight = 0.1;
  double wavePeriod = 8.;

  auto waveField = ocean->GetFreeSurface()->SetAiryRegularWaveField();
  waveField->SetWaveHeight(waveHeight);
  waveField->SetWavePeriod(wavePeriod);
  waveField->SetStretching(WHEELER); // WHEELER

  system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0., 0., wavePeriod, 1.);
  system.GetEnvironment()->GetTimeRamp()->SetActive(true);

  ocean->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(-50., 50., 5, -50., 50., 5);
  ocean->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetUpdateStep(5);
  //ocean->GetFreeSurface()->Show(true);

  //ocean->GetSeabed()->SetBathymetry(-45, NWU);
  ocean->GetSeabed()->SetBathymetry(-1000, NWU);
  ocean->GetSeabed()->GetSeabedGridAsset()->SetGrid(-50, 50, 5, -50, 50, 2);
  ocean->GetSeabed()->Show(true);

  // Body

  auto jacket = system.NewBody("jacket");
  jacket->GetDOFMask()->MakeItLocked();

  auto jacketAsset = FrFileSystem::join(
      {system.config_file().GetDataFolder(), "ce/JacketStructure/jacket.obj"});
  jacket->AddMeshAsset(jacketAsset);

  // Morison

  auto morisonFile = FrFileSystem::join(
      {system.config_file().GetDataFolder(), "ce/JacketStructure/jacket.json"});

  auto morison = make_morison_model("morison", jacket, morisonFile, true);

  auto morisonForce = make_morison_force("morisonForce", jacket, morison);

  // Buoyancy

  auto barModel = make_bar_element(jacket, morisonFile);
  auto barForce = make_buoyancy_bar_elements("buoyancy", jacket, barModel);

  // Simulation

  double dt = 0.01;
  double t_end = 10* wavePeriod;
  double time = 0.;

  system.SetTimeStep(dt);
  system.Initialize();

  bool is_irrlicht = false;

  if (is_irrlicht) {
    system.RunInViewer(t_end, 200, false);
  } else {
    while (time < t_end) {
      time += dt;
      system.AdvanceTo(time);
      std::cout << "time : " << time << " s" << std::endl;
    }
  }

}