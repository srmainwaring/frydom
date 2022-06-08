//
// Created by camille on 31/05/2021.
//

#include "frydom/frydom.h"

using namespace frydom;

int main() {

  FrOffshoreSystem system("test_CastorFile");

  // ----------------------------------------------------
  // Sea environment
  // ----------------------------------------------------

  auto FreeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();

  auto waveField = FreeSurface->SetAiryIrregularWaveField();

  double hs = 3;
  double tp = 9.;
  auto jonswap = waveField->SetJonswapWaveSpectrum(hs, tp);

  waveField->SetWaveFrequencies(0.5, 2., 20);

  waveField->SetMeanWaveDirection(Direction(SOUTH(NWU)), NWU, GOTO);

  double spreadingFactor = 10;
  unsigned int nbDir = 10;
  waveField->SetDirectionalParameters(nbDir, spreadingFactor);

  // -------------------------------------------------------
  // Body
  // -------------------------------------------------------

  auto ship = system.NewBody("ship");

  ship->AddMeshAsset("barge/barge.obj", Position(), FrRotation());

  ship->SetFixedInWorld(true);

  // --------------------------------------------------------
  // Initialization
  // --------------------------------------------------------

  system.Initialize();

}