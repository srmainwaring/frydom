//
// Created by lletourn on 01/06/2021.
//

#include "frydom/frydom.h"
#include "acme/acme.h"

using namespace frydom;

int test_FPP1Q() {

  FrOffshoreSystem system("test_FPP1Q");
  auto rho = 1025;
  system.GetEnvironment()->GetOcean()->SetDensity(rho);

  system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
  auto current = system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform();
//  current->SetNorth(1.0, MS, COMEFROM);
  current->Set(20, 1, DEG, MS, NWU, COMEFROM);

  auto body = system.NewBody("body");
  body->SetFixedInWorld(true);
//  body->SetVelocityInBodyNoRotation(Velocity(-1.0, 0., 0.), NWU);

  auto node = body->NewNode("node");

  // FPP_1Q
//  std::string open_water_data_table = R"({"j":[0,0.5,1], "kt":[1,0.5,0], "kq":[0.5,1,0.5]})";
  std::string open_water_data_table = R"({"j": [0, 0.01010101, 0.02020202, 0.03030303, 0.04040404, 0.05050505, 0.06060606, 0.07070707, 0.08080808, 0.09090909, 0.1010101, 0.11111111, 0.12121212, 0.13131313, 0.14141414, 0.15151515, 0.16161616, 0.17171717, 0.18181818, 0.19191919, 0.2020202, 0.21212121, 0.22222222, 0.23232323, 0.24242424, 0.25252525, 0.26262626, 0.27272727, 0.28282828, 0.29292929, 0.3030303, 0.31313131, 0.32323232, 0.33333333, 0.34343434, 0.35353535, 0.36363636, 0.37373737, 0.38383838, 0.39393939, 0.4040404, 0.41414141, 0.42424242, 0.43434343, 0.44444444, 0.45454545, 0.46464646, 0.47474747, 0.48484848, 0.49494949, 0.50505051, 0.51515152, 0.52525253, 0.53535354, 0.54545455, 0.55555556, 0.56565657, 0.57575758, 0.58585859, 0.5959596, 0.60606061, 0.61616162, 0.62626263, 0.63636364, 0.64646465, 0.65656566, 0.66666667, 0.67676768, 0.68686869, 0.6969697, 0.70707071, 0.71717172, 0.72727273, 0.73737374, 0.74747475, 0.75757576, 0.76767677, 0.77777778, 0.78787879, 0.7979798, 0.80808081, 0.81818182, 0.82828283, 0.83838384, 0.84848485, 0.85858586, 0.86868687, 0.87878788, 0.88888889, 0.8989899, 0.90909091, 0.91919192, 0.92929293, 0.93939394, 0.94949495, 0.95959596, 0.96969697, 0.97979798, 0.98989899, 1],
      "kt": [0.35422196, 0.35142214, 0.34857537, 0.34568216, 0.34274302, 0.33975846, 0.33672898, 0.3336551, 0.33053732, 0.32737615, 0.32417211, 0.3209257, 0.31763743, 0.31430781, 0.31093735, 0.30752656, 0.30407595, 0.30058602, 0.29705729, 0.29349026, 0.28988544, 0.28624335, 0.28256449, 0.27884937, 0.2750985,  0.27131239, 0.26749155, 0.26363649, 0.25974771, 0.25582573, 0.25187106, 0.24788419, 0.24386565, 0.23981594, 0.23573558, 0.23162506, 0.2274849,  0.22331562, 0.2191177,  0.21489168, 0.21063805, 0.20635733, 0.20205002, 0.19771663, 0.19335768, 0.18897366, 0.1845651,  0.1801325, 0.17567636, 0.17119721, 0.16669554, 0.16217186, 0.15762669, 0.15306054, 0.14847391, 0.14386731, 0.13924125, 0.13459624, 0.12993279, 0.12525141, 0.12055261, 0.1158369,  0.11110478, 0.10635677, 0.10159337, 0.09681509, 0.09202245, 0.08721595, 0.08239609, 0.0775634,  0.07271838, 0.06786154, 0.06299338, 0.05811442, 0.05322516, 0.04832612, 0.0434178,  0.03850072, 0.03357537, 0.02864228, 0.02370195, 0.01875488, 0.0138016,  0.0088426, 0.0038784, -0.0010905, -0.00606358, -0.01104034, -0.01602027, -0.02100285, -0.02598758, -0.03097396, -0.03596146, -0.04094959, -0.04593782, -0.05092567, -0.0559126, -0.06089813, -0.06588172, -0.07086289],
      "kq": [0.04336206,  0.04307767,  0.04278789,  0.04249279,  0.04219242,  0.04188681, 0.04157602,  0.04126011,  0.04093911,  0.04061307,  0.04028205,  0.03994609, 0.03960525,  0.03925957,  0.03890909,  0.03855388,  0.03819397,  0.03782941, 0.03746027,  0.03708657,  0.03670838,  0.03632574, 0.0359387,   0.0355473, 0.03515161,  0.03475165, 0.0343475,   0.03393919,  0.03352677,  0.03311029, 0.0326898, 0.03226534,  0.03183698, 0.03140475,  0.03096871,  0.0305289, 0.03008537,  0.02963817, 0.02918735,  0.02873296,  0.02827505,  0.02781366, 0.02734885,  0.02688066,  0.02640914,  0.02593434,  0.02545632,  0.02497511, 0.02449077,  0.02400334,  0.02351288,  0.02301943,  0.02252305,  0.02202377, 0.02152166,  0.02101675,  0.0205091,   0.01999876, 0.01948577,  0.01897018, 0.01845205, 0.01793141,  0.01740833, 0.01688284,  0.016355,    0.01582486, 0.01529246,  0.01475786,  0.0142211,   0.01368222,  0.01314129,  0.01259835, 0.01205344,  0.01150662,  0.01095793,  0.01040742, 0.00985515,  0.00930116, 0.0087455,  0.00818822,  0.00762936,  0.00706898,  0.00650712,  0.00594384, 0.00537918,  0.00481319,  0.00424592,  0.00367741,  0.00310773,  0.00253691, 0.001965,    0.00139206,  0.00081813,  0.00024326, -0.0003325,  -0.0009091, -0.0014865,  -0.00206464, -0.00264347, -0.00322295]
    })";


  json FPP1Q_json = {{"propeller", {
                                       {"reference", "test"},
                                       {"type", "FPP_FIRST_QUADRANT"},
                                       {"open_water_table", {
                                                                {"screw_direction", "RIGHT_HANDED"},
                                                                {"J", {0, 0.01010101, 0.02020202, 0.03030303, 0.04040404, 0.05050505, 0.06060606, 0.07070707, 0.08080808, 0.09090909, 0.1010101, 0.11111111, 0.12121212, 0.13131313, 0.14141414, 0.15151515, 0.16161616, 0.17171717, 0.18181818, 0.19191919, 0.2020202, 0.21212121, 0.22222222, 0.23232323, 0.24242424, 0.25252525, 0.26262626, 0.27272727, 0.28282828, 0.29292929, 0.3030303, 0.31313131, 0.32323232, 0.33333333, 0.34343434, 0.35353535, 0.36363636, 0.37373737, 0.38383838, 0.39393939, 0.4040404, 0.41414141, 0.42424242, 0.43434343, 0.44444444, 0.45454545, 0.46464646, 0.47474747, 0.48484848, 0.49494949, 0.50505051, 0.51515152, 0.52525253, 0.53535354, 0.54545455, 0.55555556, 0.56565657, 0.57575758, 0.58585859, 0.5959596, 0.60606061, 0.61616162, 0.62626263, 0.63636364, 0.64646465, 0.65656566, 0.66666667, 0.67676768, 0.68686869, 0.6969697, 0.70707071, 0.71717172, 0.72727273, 0.73737374, 0.74747475, 0.75757576, 0.76767677, 0.77777778, 0.78787879, 0.7979798, 0.80808081, 0.81818182, 0.82828283, 0.83838384, 0.84848485, 0.85858586, 0.86868687, 0.87878788, 0.88888889, 0.8989899, 0.90909091, 0.91919192, 0.92929293, 0.93939394, 0.94949495, 0.95959596, 0.96969697, 0.97979798, 0.98989899, 1}},
                                                                {"kt", {0.35422196, 0.35142214, 0.34857537, 0.34568216, 0.34274302, 0.33975846, 0.33672898, 0.3336551, 0.33053732, 0.32737615, 0.32417211, 0.3209257, 0.31763743, 0.31430781, 0.31093735, 0.30752656, 0.30407595, 0.30058602, 0.29705729, 0.29349026, 0.28988544, 0.28624335, 0.28256449, 0.27884937, 0.2750985, 0.27131239, 0.26749155, 0.26363649, 0.25974771, 0.25582573, 0.25187106, 0.24788419, 0.24386565, 0.23981594, 0.23573558, 0.23162506, 0.2274849, 0.22331562, 0.2191177, 0.21489168, 0.21063805, 0.20635733, 0.20205002, 0.19771663, 0.19335768, 0.18897366, 0.1845651, 0.1801325, 0.17567636, 0.17119721, 0.16669554, 0.16217186, 0.15762669, 0.15306054, 0.14847391, 0.14386731, 0.13924125, 0.13459624, 0.12993279, 0.12525141, 0.12055261, 0.1158369, 0.11110478, 0.10635677, 0.10159337, 0.09681509, 0.09202245, 0.08721595, 0.08239609, 0.0775634, 0.07271838, 0.06786154, 0.06299338, 0.05811442, 0.05322516, 0.04832612, 0.0434178, 0.03850072, 0.03357537, 0.02864228, 0.02370195, 0.01875488, 0.0138016, 0.0088426, 0.0038784, -0.0010905, -0.00606358, -0.01104034, -0.01602027, -0.02100285, -0.02598758, -0.03097396, -0.03596146, -0.04094959, -0.04593782, -0.05092567, -0.0559126, -0.06089813, -0.06588172, -0.07086289}},
                                                                {"kq", {0.04336206, 0.04307767, 0.04278789, 0.04249279, 0.04219242, 0.04188681, 0.04157602, 0.04126011, 0.04093911, 0.04061307, 0.04028205, 0.03994609, 0.03960525, 0.03925957, 0.03890909, 0.03855388, 0.03819397, 0.03782941, 0.03746027, 0.03708657, 0.03670838, 0.03632574, 0.0359387, 0.0355473, 0.03515161, 0.03475165, 0.0343475, 0.03393919, 0.03352677, 0.03311029, 0.0326898, 0.03226534, 0.03183698, 0.03140475, 0.03096871, 0.0305289, 0.03008537, 0.02963817, 0.02918735, 0.02873296, 0.02827505, 0.02781366, 0.02734885, 0.02688066, 0.02640914, 0.02593434, 0.02545632, 0.02497511, 0.02449077, 0.02400334, 0.02351288, 0.02301943, 0.02252305, 0.02202377, 0.02152166, 0.02101675, 0.0205091, 0.01999876, 0.01948577, 0.01897018, 0.01845205, 0.01793141, 0.01740833, 0.01688284, 0.016355, 0.01582486, 0.01529246, 0.01475786, 0.0142211, 0.01368222, 0.01314129, 0.01259835, 0.01205344, 0.01150662, 0.01095793, 0.01040742, 0.00985515, 0.00930116, 0.0087455, 0.00818822, 0.00762936, 0.00706898, 0.00650712, 0.00594384, 0.00537918, 0.00481319, 0.00424592, 0.00367741, 0.00310773, 0.00253691, 0.001965, 0.00139206, 0.00081813, 0.00024326, -0.0003325, -0.0009091, -0.0014865, -0.00206464, -0.00264347, -0.00322295}},
                                                            }}
                                   }}
  };

  std::string filename = "temp.json";
  std::ofstream file;
  file.open(filename, std::ios::trunc);
  file << FPP1Q_json.dump(2);
  file.close();

  double diameter_m = 2;
  double hull_wake_fraction_0 = 0.25;
  double thrust_deduction_factor_0 = 0.2;
  double rpm = 120;
//  bool use_advance_velocity_correction_factor = false;
//  double propeller_design_rpm;
//  double vessel_design_speed_ms;

  Velocity prop_relative_velocity = -system.GetEnvironment()->GetRelativeVelocityInFrame(node->GetFrameInWorld(),
                                                                                         node->GetVelocityInWorld(NWU),
                                                                                         WATER,
                                                                                         NWU);
  auto u_NWU = prop_relative_velocity.GetVx();
  auto v_NWU = prop_relative_velocity.GetVy();
  std::cout << "u_NWU :" << u_NWU << std::endl;
  std::cout << "v_NWU :" << v_NWU << std::endl;

//  auto file = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx027/propeller_FPP.json"});
  auto frydom_FPP1Q = make_first_quadrant_propeller_force("FPP1Q", body, node->GetNodePositionInBody(NWU), filename,
                                                          NWU);
  frydom_FPP1Q->SetDiameter(diameter_m);
  frydom_FPP1Q->SetScrewDirection(SCREW_DIRECTION::RIGHT_HANDED);
  frydom_FPP1Q->SetRotationalVelocity(rpm, RPM);
  frydom_FPP1Q->SetStraightRunWakeFraction(hull_wake_fraction_0);
  frydom_FPP1Q->SetThrustDeductionFactor(thrust_deduction_factor_0);

  acme::ThrusterBaseParams params(diameter_m, hull_wake_fraction_0, thrust_deduction_factor_0,
                                  acme::SCREW_DIRECTION::RIGHT_HANDED);
//  acme::ThrusterBaseParams params;
//  params.m_diameter_m = diameter_m;
//  params.m_screw_direction = acme::SCREW_DIRECTION::RIGHT_HANDED;
//  params.m_hull_wake_fraction_0 = hull_wake_fraction_0;
//  params.m_thrust_deduction_factor_0 = thrust_deduction_factor_0;
//  params.m_propeller_design_rpm = propeller_design_rpm;
//  params.m_vessel_design_speed_ms = vessel_design_speed_ms;

  auto acme_FPP1Q = acme::FPP1Q(params, open_water_data_table);

  system.Initialize();
  acme_FPP1Q.Initialize();
  acme_FPP1Q.Compute(rho, u_NWU, v_NWU, rpm, 0.);
  std::cout << "ACME Thrust :" << acme_FPP1Q.GetThrust() << std::endl;
  std::cout << "ACME Torque :" << acme_FPP1Q.GetTorque() << std::endl;
  std::cout << "ACME Power :" << acme_FPP1Q.GetPower() << std::endl;
  std::cout << "ACME Efficiency :" << acme_FPP1Q.GetPropellerEfficiency() << std::endl;

  std::cout << "FRyDoM Thrust :" << frydom_FPP1Q->GetThrust() << std::endl;
  std::cout << "FRyDoM Torque :" << frydom_FPP1Q->GetTorque() << std::endl;
  std::cout << "FRyDoM Power :" << frydom_FPP1Q->GetPower() << std::endl;


//  auto file = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx022/propeller_BCP_1320f.json"});
//  auto force = make_controllable_pitch_propeller("CPPForce", body.get(), Position(), file, NWU);

//  auto file = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx022/propeller_FPP_template.json"});
//  auto fpp = std::make_shared<FrFirstQuadrantPropellerForce>("FPP", body.get(), Position(), file, NWU);
//  body->AddExternalForce(fpp);

//  auto file = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx022/flap_rudder.json"});
//  auto rudder = make_flap_rudder_force("rudder", body, node, file);
//  rudder->SetFlapLaw(0.8);
//  rudder->SetRampSlope(1, DEGS);
//  rudder->SetProjectedLateralArea(15.061);
//
//  rudder->SetRudderAngle(20, DEG);

//  system.Initialize();
//
//  system.AdvanceTo(30);

//  force->SetPitchRatio(1.5);
//  std::cout<<"Ct="<<force->Ct(-180*DEG2RAD)<<", Cq="<<force->Cq(-180*DEG2RAD)<<std::endl;

//  std::cout << "kt = " << fpp->kt(1.) << ", kq = " << fpp->kq(1.) << std::endl;
//
//  auto inflowVelocity = rudder->GetRudderRelativeVelocityInWorld();
//  auto attack_angle = rudder->GetAttackAngle(-inflowVelocity);
//  std::cout << "rudder inflow velocity : " << inflowVelocity << std::endl;
//  std::cout << "rudder angle : " << rudder->GetRudderAngle(DEG) << std::endl;
//  std::cout << "rudder drift angle : " << rudder->GetDriftAngle(-inflowVelocity) * RAD2DEG << std::endl;
//  std::cout << "rudder attack angle : " << attack_angle * RAD2DEG << std::endl;
//
//
//  std::cout << "cd = "    << rudder->GetDragCoefficient(attack_angle)
//            << ", cl = "  << rudder->GetLiftCoefficient(attack_angle)
//            << ", cn = "  << rudder->GetTorqueCoefficient(attack_angle) << std::endl;
//
//  std::cout << "rudder force : " << rudder->GetForceInWorld(NWU) << std::endl;
//  std::cout << "rudder torque : " << rudder->GetTorqueInWorldAtCOG(NWU) << std::endl;

//  auto X = force->GetCoeff().GetX();
//  std::cout<<"X : "<<Eigen::VectorXd::Map(&X[0], X.size())<<std::endl;
//  auto Y = force->GetCoeff().GetY();
//  std::cout<<"Y : "<<Eigen::VectorXd::Map(&Y[0], Y.size())<<std::endl;

  return 0;

}

int test_FPP4Q() {

  FrOffshoreSystem system("test_FPP4Q");
  auto rho = 1025;
  system.GetEnvironment()->GetOcean()->SetDensity(rho);

  system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
  auto current = system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform();
//  current->SetNorth(-1.0, MS, COMEFROM);
  current->Set(20, 1, DEG, MS, NWU, COMEFROM);
  double rpm = 120;

  auto body = system.NewBody("body");
  body->SetFixedInWorld(true);
//  body->SetVelocityInBodyNoRotation(Velocity(-1.0, 0., 0.), NWU);

  auto node = body->NewNode("node");

  // FPP_4Q
  std::string open_water_data_table = R"({"beta_deg": [-180.0,-140.0,-100.0,-60.00000000000001,-20.000000000000004,20.000000000000004,59.999999999999986,99.99999999999999,140.0,180.0],
      "ct": [-0.16268168999999996,0.4486688377385315,0.8944023203183783,0.8032479305281882,0.27572401377867956,0.017939934846272564,-1.142724540528188,-1.3839567584729406,-0.7788844482089202,-0.16268169000000005],
      "cq": [-0.018609981000000005,0.055124843060058874,0.10925670286110463,0.1255048384078987,0.05556630922628392,0.015411367752487179,-0.10401960640789869,-0.12289300851577142,-0.0810090123841631,-0.018609980999999987]
    })";


  json FPP4Q_json = {{"propeller", {
                                       {"reference", "test"},
                                       {"type", "FPP_FOUR_QUADRANTS"},
                                       {"open_water_table", {
                                                                {"screw_direction", "RIGHT_HANDED"},
                                                                {"beta_deg", {-180.0, -140.0, -100.0, -60.00000000000001, -20.000000000000004, 20.000000000000004, 59.999999999999986, 99.99999999999999, 140.0, 180.0}},
                                                                {"ct", {-0.16268168999999996, 0.4486688377385315, 0.8944023203183783, 0.8032479305281882, 0.27572401377867956, 0.017939934846272564, -1.142724540528188, -1.3839567584729406, -0.7788844482089202, -0.16268169000000005}},
                                                                {"cq", {-0.018609981000000005, 0.055124843060058874, 0.10925670286110463, 0.1255048384078987, 0.05556630922628392, 0.015411367752487179, -0.10401960640789869, -0.12289300851577142, -0.0810090123841631, -0.018609980999999987}}
                                                            }}
                                   }}
  };

  std::string filename = "temp.json";
  std::ofstream file;
  file.open(filename, std::ios::trunc);
  file << FPP4Q_json.dump(2);
  file.close();

  double diameter_m = 2;
  double hull_wake_fraction_0 = 0.2;
  double thrust_deduction_factor_0 = 0.25;
//  bool use_advance_velocity_correction_factor = false;
//  double propeller_design_rpm;
//  double vessel_design_speed_ms;

  Velocity prop_relative_velocity = -system.GetEnvironment()->GetRelativeVelocityInFrame(node->GetFrameInWorld(),
                                                                                         node->GetVelocityInWorld(NWU),
                                                                                         WATER,
                                                                                         NWU);
  auto u_NWU = prop_relative_velocity.GetVx();
  auto v_NWU = prop_relative_velocity.GetVy();
  std::cout << "u_NWU :" << u_NWU << std::endl;
  std::cout << "v_NWU :" << v_NWU << std::endl;

//  auto file = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx027/propeller_FPP.json"});
  auto frydom_FPP4Q = make_four_quadrant_propeller_force("FPP4Q", body, node->GetNodePositionInBody(NWU), filename,
                                                         NWU);
  frydom_FPP4Q->SetDiameter(diameter_m);
  frydom_FPP4Q->SetScrewDirection(SCREW_DIRECTION::RIGHT_HANDED);
  frydom_FPP4Q->SetRotationalVelocity(rpm, RPM);
  frydom_FPP4Q->SetStraightRunWakeFraction(hull_wake_fraction_0);
  frydom_FPP4Q->SetThrustDeductionFactor(thrust_deduction_factor_0);

  acme::ThrusterBaseParams params(diameter_m, hull_wake_fraction_0, thrust_deduction_factor_0,
                                  acme::SCREW_DIRECTION::RIGHT_HANDED);

  auto acme_FPP4Q = acme::FPP4Q(params, open_water_data_table);

  system.Initialize();
  acme_FPP4Q.Initialize();
  acme_FPP4Q.Compute(rho, u_NWU, v_NWU, rpm, 0.);
  std::cout << "beta : " << frydom_FPP4Q->ComputeAdvanceAngle() << std::endl;
  std::cout << "ACME Thrust :" << acme_FPP4Q.GetThrust() << std::endl;
  std::cout << "ACME Torque :" << acme_FPP4Q.GetTorque() << std::endl;
  std::cout << "ACME Power :" << acme_FPP4Q.GetPower() << std::endl;
  std::cout << "ACME Efficiency :" << acme_FPP4Q.GetPropellerEfficiency() << std::endl;

  std::cout << "FRyDoM Thrust :" << frydom_FPP4Q->GetThrust() << std::endl;
  std::cout << "FRyDoM Torque :" << frydom_FPP4Q->GetTorque() << std::endl;
  std::cout << "FRyDoM Power :" << frydom_FPP4Q->GetPower() << std::endl;

  return 0;

}


int main() {
  return test_FPP4Q();
}