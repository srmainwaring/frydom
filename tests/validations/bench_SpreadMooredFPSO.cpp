//
// Created by camille on 24/08/2020.
//

#include "frydom/frydom.h"

using namespace frydom;

int main(int agrc, char* argv[]) {

  // System
  FrOffshoreSystem system("spread_moored_FPSO");
  system.GetEnvironment()->SetGravityAcceleration(9.807);

  /*
  system.SetSolver(FrOffshoreSystem::SOLVER::MINRES);
  system.SetSolverWarmStarting(true);
  system.SetSolverMaxIterSpeed(1000);
  system.SetSolverMaxIterStab(1000);
  system.SetSolverForceTolerance(1e-8);
  system.SetSolverDiagonalPreconditioning(true);
  */

  // Ocean
  auto ocean = system.GetEnvironment()->GetOcean();
  ocean->SetDensity(1000);

  auto waveField = ocean->GetFreeSurface()->SetAiryRegularWaveField();
  waveField->SetWaveHeight(0.);
  waveField->SetWavePeriod(10.);
  waveField->SetDirection(180., DEG, NWU, GOTO);

  system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0., 0., 10., 1.);

  ocean->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(
      -1600, 1600, 20,
      -800, 800, 20);
  ocean->GetFreeSurface()->GetFreeSurfaceGridAsset()->UpdateAssetON();
  ocean->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetUpdateStep(10);

  auto seabed = system.GetEnvironment()->GetOcean()->GetSeabed();
  seabed->SetBathymetry(-1335, NWU);

  seabed->GetSeabedGridAsset()->SetGrid(-1600, 1600, 100);
  seabed->SetBathymetry(-1335., NWU);
  seabed->Show(true);

  // FPSO

  auto body = system.NewBody("FPSO");
  body->SetPosition(Position(0., 0., -15.7), NWU);
  body->AddBoxShape(300., 60., 33.2, {0., 0., 15.7}, NWU);
  //body->SetFixedInWorld(false);
  body->GetDOFMask()->SetLock_X(true);
  body->GetDOFMask()->SetLock_Y(true);
  body->GetDOFMask()->SetLock_Z(false);
  body->GetDOFMask()->SetLock_Rx(true);
  body->GetDOFMask()->SetLock_Ry(true);
  body->GetDOFMask()->SetLock_Rz(true);

  auto node0 = body->NewNode("node0");
  node0->SetPositionInBody({0., 0., 15.7}, NWU);

  // Inertia

  double mass = 2.7364774e8;

  double Ixx = 1.521e11;
  double Iyy = 1.667e12;
  double Izz = 1.756e12;

  Position COGPosition(7.27, 0., 20.111);
  //Position COGPosition(0., 0., 0.);

  FrInertiaTensor InertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., COGPosition, NWU);
  body->SetInertiaTensor(InertiaTensor);

  // Hydrodynamic

  auto hdb_file = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/bench/FPSO/FPSO_2.hdb5"});
  auto hdb = make_hydrodynamic_database(hdb_file);

  auto eqFrame = make_equilibrium_frame("EqFrame", body, {7.27, 0., 20.111}, NWU);
  hdb->Map(0, body.get(), eqFrame);

  // Hydrostatic
  auto forceHst = make_linear_hydrostatic_force("linear_hydrostatic", body, hdb);

  // Radiation
  auto radiationModel = make_radiation_convolution_model("radiation_convolution", &system, hdb);

  // Excitation
  auto excitationForce = make_linear_excitation_force("linear_excitation", body, hdb);

  // Wave drift
  //auto waveDriftForce = make_wave_drift_force("wave_drift", body, hdb);

  bool is_moored = false;

  if (is_moored) {

    // Anchors
    Position refPos = {0., 0., 0.};
    //auto anchor_P1 = seabed->NewAnchor("anchor_P1", refPos, 141.702, 1772.629, DEG, NWU);
    auto anchor_P1 = seabed->NewAnchor("anchor_P1", -1383.71, 1097.96, NWU);
    auto anchor_P2 = seabed->NewAnchor("anchor_P2", refPos, 136.892, 1763.576, DEG, NWU);
    auto anchor_P3 = seabed->NewAnchor("anchor_P3", refPos, 132.130, 1748.153, DEG, NWU);
    auto anchor_P4 = seabed->NewAnchor("anchor_P4", refPos, 47.947, 1703.139, DEG, NWU);
    auto anchor_P5 = seabed->NewAnchor("anchor_P5", refPos, 43.112, 1714.879, DEG, NWU);
    auto anchor_P6 = seabed->NewAnchor("anchor_P6", refPos, 38.258, 1717.236, DEG, NWU);
    auto anchor_S1 = seabed->NewAnchor("anchor_S1", refPos, 218.144, 1566.337, DEG, NWU);
    auto anchor_S2 = seabed->NewAnchor("anchor_S2", refPos, 222.950, 1566.576, DEG, NWU);
    auto anchor_S3 = seabed->NewAnchor("anchor_S3", refPos, 227.664, 1548.130, DEG, NWU);
    auto anchor_S4 = seabed->NewAnchor("anchor_S4", refPos, 312.321, 1526.204, DEG, NWU);
    auto anchor_S5 = seabed->NewAnchor("anchor_S5", refPos, 317.012, 1534.292, DEG, NWU);
    auto anchor_S6 = seabed->NewAnchor("anchor_S6", refPos, 321.766, 1538.778, DEG, NWU);

    double dz0 = 3.07;
    anchor_P1->TranslateInWorld({0., 0., dz0}, NWU);
    anchor_P2->TranslateInWorld({0., 0., dz0}, NWU);
    anchor_P3->TranslateInWorld({0., 0., dz0}, NWU);
    anchor_P4->TranslateInWorld({0., 0., dz0}, NWU);
    anchor_P5->TranslateInWorld({0., 0., dz0}, NWU);
    anchor_P6->TranslateInWorld({0., 0., dz0}, NWU);
    anchor_S1->TranslateInWorld({0., 0., dz0}, NWU);
    anchor_S2->TranslateInWorld({0., 0., dz0}, NWU);
    anchor_S3->TranslateInWorld({0., 0., dz0}, NWU);
    anchor_S4->TranslateInWorld({0., 0., dz0}, NWU);
    anchor_S5->TranslateInWorld({0., 0., dz0}, NWU);
    anchor_S6->TranslateInWorld({0., 0., dz0}, NWU);

    // Fairlead
    auto fairlead_S1 = body->NewNode("fairlead_S1");
    fairlead_S1->SetPositionInBody({-112.911003, -31.636, 6.499}, NWU);
    auto fairlead_S2 = body->NewNode("fairlead_S2");
    fairlead_S2->SetPositionInBody({-109.981003, -31.636, 6.473}, NWU);
    auto fairlead_S3 = body->NewNode("fairlead_S3");
    fairlead_S3->SetPositionInBody({-107.051003, -31.636, 6.448}, NWU);
    auto fairlead_S4 = body->NewNode("fairlead_S4");
    fairlead_S4->SetPositionInBody({108.865997, -31.636, 4.576}, NWU);
    auto fairlead_S5 = body->NewNode("fairlead_S5");
    fairlead_S5->SetPositionInBody({111.795998, -31.636, 4.551}, NWU);
    auto fairlead_S6 = body->NewNode("fairlead_S6");
    fairlead_S6->SetPositionInBody({114.75998, -31.636, 4.526}, NWU);
    auto fairlead_P1 = body->NewNode("fairlead_P1");
    fairlead_P1->SetPositionInBody({-112.911003, 31.636, 6.499}, NWU);
    auto fairlead_P2 = body->NewNode("fairlead_P2");
    fairlead_P2->SetPositionInBody({-109.981003, 31.636, 6.473}, NWU);
    auto fairlead_P3 = body->NewNode("fairlead_P3");
    fairlead_P3->SetPositionInBody({-107.051003, 31.636, 6.448}, NWU);
    auto fairlead_P4 = body->NewNode("fairlead_P4");
    fairlead_P4->SetPositionInBody({108.865997, 31.636, 4.576}, NWU);
    auto fairlead_P5 = body->NewNode("fairlead_P5");
    fairlead_P5->SetPositionInBody({111.795998, 31.636, 4.551}, NWU);
    auto fairlead_P6 = body->NewNode("fairlead_P6");
    fairlead_P6->SetPositionInBody({114.752998, 31.636, 4.526}, NWU);

    // Mooring lines

    auto cableProp = make_cable_properties();
    cableProp->SetDiameter(0.1);
    cableProp->SetLinearDensity(50);
    cableProp->SetEA(9.18e8);
    cableProp->SetBreakingTension(1.0266e7);
    cableProp->SetDragCoefficients(1.2, 0.);
    cableProp->SetAddedMassCoefficients(1., 0.);
    cableProp->SetHydrodynamicDiameter(0.1);
    cableProp->SetRayleighDamping(0.);
    cableProp->SetVIVAmpFactor(0.);

    unsigned int nbElements = 100;

    // Catenary Cable


    auto line_P1 = make_catenary_line("line_P1", anchor_P1, fairlead_P1, cableProp,
                                      true, 2141.460, WATER);

    auto line_P2 = make_catenary_line("line_P2", anchor_P2, fairlead_P2, cableProp,
                                      true, 2138.360, WATER);

    auto line_P3 = make_catenary_line("line_P3", anchor_P3, fairlead_P3, cableProp,
                                      true, 2141.160, WATER);

    auto line_P4 = make_catenary_line("line_P4", anchor_P4, fairlead_P4, cableProp,
                                      true, 2068.100, WATER);

    auto line_P5 = make_catenary_line("line_P5", anchor_P5, fairlead_P5, cableProp,
                                      true, 2068.000, WATER);

    auto line_P6 = make_catenary_line("line_P6", anchor_P6, fairlead_P6, cableProp,
                                      true, 2068.800, WATER);

    auto line_S1 = make_catenary_line("line_S1", anchor_S1, fairlead_S1, cableProp,
                                      true, 1965.470, WATER);

    auto line_S2 = make_catenary_line("line_S2", anchor_S2, fairlead_S2, cableProp,
                                      true, 1964.870, WATER);

    auto line_S3 = make_catenary_line("line_S3", anchor_S3, fairlead_S3, cableProp,
                                      true, 1965.670, WATER);

    auto line_S4 = make_catenary_line("line_S4", anchor_S4, fairlead_S4, cableProp,
                                      true, 1907.300, WATER);

    auto line_S5 = make_catenary_line("line_S5", anchor_S5, fairlead_S5, cableProp,
                                      true, 1907.400, WATER);

    auto line_S6 = make_catenary_line("line_S6", anchor_S6, fairlead_S6, cableProp,
                                      true, 1907.500, WATER);

    // Fea Cable

    /*
    auto line_fea_P1 = make_fea_cable("line_fea_P1", anchor_P1, fairlead_P1, cableProp,
                                      2141.460, nbElements);

    auto line_fea_P2 = make_fea_cable("line_fea_P2", anchor_P2, fairlead_P2, cableProp,
                                      2138.360, nbElements);

    auto line_fea_P3 = make_fea_cable("line_fea_P3", anchor_P3, fairlead_P3, cableProp,
                                      2141.160, nbElements);

    auto line_fea_P4 = make_fea_cable("line_fea_P4", anchor_P4, fairlead_P4, cableProp,
                                      2068.100, nbElements);

    auto line_fea_P5 = make_fea_cable("line_fea_P5", anchor_P5, fairlead_P5, cableProp,
                                      2068.000, nbElements);

    auto line_fea_P6 = make_fea_cable("line_fea_P6", anchor_P6, fairlead_P6, cableProp,
                                      2068.800, nbElements);

    auto line_fe+a_S1 = make_fea_cable("line_fea_S1", anchor_S1, fairlead_S1, cableProp,
                                      1965.470, nbElements);

    auto line_fea_S2 = make_fea_cable("line_fea_S2", anchor_S2, fairlead_S2, cableProp,
                                      1964.870, nbElements);

    auto line_fea_S3 = make_fea_cable("line_fea_S3", anchor_S3, fairlead_S3, cableProp,
                                      1965.670, nbElements);

    auto line_fea_S4 = make_fea_cable("line_fea_S4", anchor_S4, fairlead_S4, cableProp,
                                       1907.300, nbElements);

    auto line_fea_S5 = make_fea_cable("line_fea_S5", anchor_S5, fairlead_S5, cableProp,
                                       1907.400, nbElements);

    auto line_fea_S6 = make_fea_cable("line_fea_S6", anchor_S6, fairlead_S6, cableProp,
                                      1907.500, nbElements);
    */

  }

  // Simulation

  double dt = 0.05;
  double t_max = 600.;

  system.SetTimeStep(dt);
  system.Initialize();

  body->TranslateInWorld(0., 0., 2., NWU);

  //system.GetStaticAnalysis()->SetNbIteration(100);
  //system.GetStaticAnalysis()->SetNbSteps(5);
  //system.SolveStaticWithRelaxation();
  //system.Visualize(1000);

  bool is_irrlicht = true;

  if (is_irrlicht) {
    system.RunInViewer(t_max, 1000, false);
  } else {
    double time = 0;
    while (time < t_max) {
      time += dt;
      system.AdvanceTo(time);
      std::cout << "time : " << time << std::endl;
    }
  }


}