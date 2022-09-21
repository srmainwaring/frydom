//
// Created by camille on 08/08/2022.
//

#include "frydom/frydom.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

using namespace frydom;

// ----------------------------------------------------------------------------
// Environment
// ----------------------------------------------------------------------------

void SetRegularWaveField(FrOffshoreSystem& system) {

  auto FreeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();

  auto waveField = FreeSurface->SetAiryRegularOptimWaveField();

  // The Airy regular wave p arameters are its height, period and direction.
  double waveHeight = 1.; //0.25
  double wavePeriod = 2. * M_PI;
  Direction waveDirection = Direction(SOUTH(NWU));

  waveField->SetWaveHeight(waveHeight);
  waveField->SetWavePeriod(wavePeriod);
  waveField->SetDirection(waveDirection, NWU, GOTO);

}

void SetIrregularWaveField(FrOffshoreSystem& system) {

  auto FreeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();

  //auto waveField = FreeSurface->SetAiryIrregularOptimWaveField();
  auto waveField = FreeSurface->SetAiryIrregularWaveField();

  //waveField->SetJonswapWaveSpectrum(1., 2.*M_PI);
  //waveField->SetMeanWaveDirection(Direction(SOUTH(NWU)), NWU, GOTO);
  //waveField->SetWaveFrequencies(0.05, 2.9, 20);
  //waveField->SetDirectionalParameters(10, 10);

  auto seastate = FrFileSystem::join(
      {system.config_file().GetDataFolder(), "test_FEACableBuoy_irregular_wave.json"});
  waveField->LoadJSON(seastate);

}

void SetEnvironment(FrOffshoreSystem& system) {

  // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
  FRAME_CONVENTION fc = NWU;
  // Define the wave direction convention (GOTO or COMEFROM), can be used also for current and wind direction definition.
  DIRECTION_CONVENTION dc = GOTO;

  system.GetEnvironment()->GetTimeRamp()->SetActive(true);
  system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0., 0., 10., 1.);

  double Bathy = -30;
  system.GetEnvironment()->GetOcean()->GetSeabed()->SetBathymetry(Bathy, NWU);

  auto SeabedGridAsset = system.GetEnvironment()->GetOcean()->GetSeabed()->GetSeabedGridAsset();
  SeabedGridAsset->SetGrid(-200., 200., 5., -50., 50., 5.);
  system.GetEnvironment()->GetOcean()->GetSeabed()->Show(true);

  /*
  auto seabedCollision = std::make_shared<FrCollisionModel>();
  auto mat = std::make_shared<FrContactParamsSMC>();
  seabedCollision->AddBox(mat.get(), 200, 50, 2, Position(0., 0., Bathy - 2), FrRotation());
  seabedCollision->BuildModel();
  system.GetWorldBody()->SetCollisionModel(seabedCollision);

  auto steel = system.GetWorldBody()->GetContactParamsSMC();
  steel->young_modulus = 1e7;
  system.GetWorldBody()->SetContactParamsSMC(steel);
  */

  auto FreeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();
  // To manipulate the free surface grid asset, you first need to access it, through the free surface object.
  auto FSAsset = system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset();

  // The free surface grid is defined here as a squared one ranging from -100m to 100m (in north and west
  // directions) with a 2m steps.
  FSAsset->SetGrid(-200., 200, 2, -50, 50, 2);

  // You have to specify if you want the free surface asset to be updated during the simulation. By default, the
  // update is not activated.
  FSAsset->SetUpdateStep(10);

  // Wave field
  //SetRegularWaveField(system);
  SetIrregularWaveField(system);

}



std::shared_ptr<FrBody> SetBarge(FrOffshoreSystem& system) {

  // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
  FRAME_CONVENTION fc = NWU;
  // Define the wave direction convention (GOTO or COMEFROM), can be used also for current and wind direction definition.
  DIRECTION_CONVENTION dc = GOTO;

  auto steel = std::make_shared<FrMaterialSurfaceSMC>();
  steel->young_modulus = 1e8;

  auto barge = system.NewBody("barge");
  auto bargeMesh = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/barge/barge.obj"});
  barge->AddMeshAsset(bargeMesh);
  barge->SetColor(Yellow);


  auto collisionModel = std::make_shared<FrCollisionModel>();
  collisionModel->AddBox(steel.get(), 17.5, 8, 2, Position(), FrRotation());
//    collisionModel->AddTriangleMesh("barge.obj",Position(),FrRotation());
  collisionModel->Initialize();
  barge->SetCollisionModel(collisionModel);

  auto rev1_barge_node = barge->NewNode("rev1_barge_node");
  rev1_barge_node->SetPositionInBody(Position(-7.5, 0., 3.), fc);

  barge->SetInertiaTensor(
      FrInertiaTensor((1137.6 - 180.6) * 1000, 2.465e7, 1.149e7, 1.388e07, 0., 0., 0., Position(), NWU));

  // -- Hydrodynamics

  auto bargeHDB = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/barge/barge.hdb5"});
  auto hdb = make_hydrodynamic_database(bargeHDB);

  auto eqFrame = make_equilibrium_frame("EqFrame", barge);
  eqFrame->LogThis(true);

  hdb->Map(0, barge.get(), eqFrame);

  // Hydrostatic
  auto hydrostaticForce = make_linear_hydrostatic_force("linear_hydrostatic", barge, hdb);

  // Excitation force
  auto excitationForce = make_linear_excitation_force("linear_excitation", barge, hdb);

  // -- Radiation
  auto radiationModel = make_recursive_convolution_model("radiation_convolution", &system, hdb);
  //radiationModel->SetImpulseResponseSize(barge.get(), 20., 0.025);

  return barge;
}

void SetMooringLines(FrOffshoreSystem& system, std::shared_ptr<FrBody> barge) {

  double buoyRadius = 2.5;
  double buoyMass = 100.;
  double buoyDamping = 1000.;

  auto buoyS = make_mooring_buoy("buoyS", &system, buoyRadius, buoyMass, buoyDamping);
  buoyS->SetPosition(Position(-56., 0., 0.), NWU);
  auto buoyNodeS = buoyS->NewNode("buoyNodeS");
  //##CC debug
  //buoySE->SetFixedInWorld(true);
  //##
  auto buoyN = make_mooring_buoy("buoyN", &system, buoyRadius, buoyMass, buoyDamping);
  buoyN->SetPosition(Position(56., 0., 0.), NWU);
  auto buoyNodeN = buoyN->NewNode("buoyNodeN");


  auto worldNodeS = system.GetWorldBody()->NewNode("worldNodeS");
  worldNodeS->SetPositionInBody(Position(-145.8, 00, -30), NWU);

  auto worldNodeN = system.GetWorldBody()->NewNode("worldNodeN");
  worldNodeN->SetPositionInBody(Position(145.8, 00, -30), NWU);

  auto bargeNodeS = barge->NewNode("bargeNodeS");
  bargeNodeS->SetPositionInBody(Position(-17.5, 0., 0.), NWU);

  auto bargeNodeN = barge->NewNode("bargeNodeN");
  bargeNodeN->SetPositionInBody(Position(17.5, 0., 0.), NWU);

  bool elastic = true;
  double anchoringLineLength = 95.;
  int nb_elements = 20;
  int spline_order = 1;

  double buoyLineLength = 42.5;

  auto cableProp = make_cable_properties();
  cableProp->SetLinearDensity(600); //600 kg/m
  cableProp->SetSectionArea(0.5);
  cableProp->SetEA(5e7); //5e7 N
  cableProp->SetDragCoefficients(1., 0.);
  cableProp->SetAddedMassCoefficients(1., 0.);

  auto mooringLineS = make_fea_cable("mooringLineS", worldNodeS, buoyNodeS, cableProp, anchoringLineLength,
                                      nb_elements, spline_order);

  //auto tetherLineS = make_catenary_line("tetherLineS", bargeNodeS, buoyNodeS, cableProp, elastic, buoyLineLength,
  //                                       FLUID_TYPE::WATER);
  auto tetherLineS = make_fea_cable("tetherLineS", bargeNodeS, buoyNodeS, cableProp, buoyLineLength,
                                    10, 1);

  auto mooringLineN = make_fea_cable("mooringLineN", worldNodeN, buoyNodeN, cableProp, anchoringLineLength,
                                     nb_elements, spline_order);

  //auto tetherLineN = make_catenary_line("tetherLineN", bargeNodeN, buoyNodeN, cableProp, elastic, buoyLineLength,
  //                                      FLUID_TYPE::WATER);
  auto tetherLineN = make_fea_cable("tetherLineN", bargeNodeN, buoyNodeN, cableProp, buoyLineLength,
                                    10, 1);

}

int main(int argc, char* argv[]) {

  FrOffshoreSystem system("FEACableBuoy",
                          FrOffshoreSystem::SYSTEM_TYPE::SMOOTH_CONTACT,
                          FrOffshoreSystem::EULER_IMPLICIT_LINEARIZED,
                          FrOffshoreSystem::MINRES);

  //auto mkl_solver = std::make_shared<chrono::ChSolverPardisoMKL>();
  //system.GetChronoSystem()->SetSolver(mkl_solver);
  //mkl_solver->UseSparsityPatternLearner(true);
  //mkl_solver->LockSparsityPattern(true);

  auto slu_solver = std::make_shared<chrono::ChSolverSparseLU>();
  slu_solver->UseSparsityPatternLearner(true);
  //system.GetChronoSystem()->SetSolver(slu_solver);

  // Environment
  SetEnvironment(system);

  // Barge
  auto barge = SetBarge(system);
  //##CC debug
  //barge->SetFixedInWorld(true);
  //##

  // Mooring
  SetMooringLines(system, barge);

  // Solver properties
  system.SetSolverMaxIterations(1000);
  system.SetSolverForceTolerance(1e-7);

  auto dt = 0.01;
  system.SetTimeStep(dt);

  // Init.
  system.Initialize();

  // Static
  if (false) {
    system.GetStaticAnalysis()->SetNbIteration(50);
    system.GetStaticAnalysis()->SetNbSteps(20);
    system.SolveStaticWithRelaxation();
  }

  // Dynamic simulation

  bool is_irrlicht = false;

  clock_t begin = clock();
  auto c_start = std::chrono::high_resolution_clock::now();

  if (is_irrlicht) {
    system.RunInViewer(100, 175, false, 10);
  } else {
    auto time = 0.;
    while (time < 12.) {
      time += dt;
      system.AdvanceTo(time);
      std::cout << "Time : " << time << " s" << std::endl;
    }
  }
  clock_t end = clock();
  auto c_end = std::chrono::high_resolution_clock::now();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(c_end-c_start);
  std::cout << "Elapsed cpu time in seconds : " << elapsed_secs << std::endl;
  std::cout << "Elapsed cpu time in ms (high_resolution_clock) : " << duration.count()/1000. << std::endl;
  std::cout << "============================== End ===================================== " << std::endl;

  return 0;

}
