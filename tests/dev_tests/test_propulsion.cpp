//
// Created by lletourn on 01/06/2021.
//

#include "frydom/frydom.h"
#include "acme/acme.h"

using namespace frydom;

template<typename T>
std::string str(T begin, T end) {
  std::stringstream ss;
  bool first = true;
  ss << "[";
  for (; begin != end; begin++) {
    if (!first)
      ss << ", ";
    ss << *begin;
    first = false;
  }
  ss << "]";
  return ss.str();
}

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
//  auto u_NWU = prop_relative_velocity.GetVx();
//  auto v_NWU = prop_relative_velocity.GetVy();
//  std::cout << "u_NWU :" << u_NWU << std::endl;
//  std::cout << "v_NWU :" << v_NWU << std::endl;

//  auto file = FrFileSystem::join({system.config_file().GetDataFolder(), "rdx027/propeller_FPP.json"});
  auto frydom_FPP1Q = make_first_quadrant_propeller_force("FPP1Q", body, node->GetNodePositionInBody(NWU), filename,
                                                          NWU);
  frydom_FPP1Q->SetDiameter(diameter_m);
  frydom_FPP1Q->SetScrewDirection(SCREW_DIRECTION::RIGHT_HANDED);
  frydom_FPP1Q->SetRotationalVelocity(rpm, RPM);
  frydom_FPP1Q->SetStraightRunWakeFraction(hull_wake_fraction_0);
  frydom_FPP1Q->SetThrustDeductionFactor(thrust_deduction_factor_0);

  acme::PropellerParams params(diameter_m, hull_wake_fraction_0, thrust_deduction_factor_0,
                               acme::SCREW_DIRECTION::RIGHT_HANDED);
//  acme::ThrusterBaseParams params;
//  params.m_diameter_m = diameter_m;
//  params.m_screw_direction = acme::SCREW_DIRECTION::RIGHT_HANDED;
//  params.m_hull_wake_fraction_0 = hull_wake_fraction_0;
//  params.m_thrust_deduction_factor_0 = thrust_deduction_factor_0;
//  params.m_propeller_design_rpm = propeller_design_rpm;
//  params.m_vessel_design_speed_ms = vessel_design_speed_ms;

  auto acme_FPP1Q = make_ACME_propeller("ACME_FPP1Q", node, params, open_water_data_table, PropellerType::E_FPP1Q);
  acme_FPP1Q->SetRPM(rpm);

//  auto acme_FPP1Q = acme::FPP1Q(params, open_water_data_table);

  system.Initialize();
//  acme_FPP1Q.Compute(rho, u_NWU, v_NWU, rpm, 0.);
  std::cout << "ACME Thrust :" << acme_FPP1Q->GetThrust() << std::endl;
  std::cout << "ACME Torque :" << acme_FPP1Q->GetTorque() << std::endl;
  std::cout << "ACME Power :" << acme_FPP1Q->GetPower() << std::endl;
  std::cout << "ACME Efficiency :" << acme_FPP1Q->GetEfficiency() << std::endl;

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
  current->SetNorth(-1.0, MS, COMEFROM);
//  current->Set(20, 1, DEG, MS, NWU, COMEFROM);
  double rpm = 600;

  auto body = system.NewBody("body");
  body->SetFixedInWorld(true);

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

  acme::PropellerParams params(diameter_m, hull_wake_fraction_0, thrust_deduction_factor_0,
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

int test_CPP() {

  FrOffshoreSystem system("test_CPP");
  auto rho = 1025;
  system.GetEnvironment()->GetOcean()->SetDensity(rho);

  system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
  auto current = system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform();
  current->SetNorth(1.0, MS, COMEFROM);
//  current->Set(20, 1, DEG, MS, NWU, COMEFROM);
  double rpm = 60;
  double pitch_ratio = 0.;

  auto body = system.NewBody("body");
  body->SetFixedInWorld(true);

  auto node = body->NewNode("node");

  // CPP
  std::string open_water_data_table = R"({"beta_deg": [-180.0,-140.0,-100.0,-60.00000000000001,-20.000000000000004,20.000000000000004,59.999999999999986,99.99999999999999,140.0,180.0],
      "p_d": [ -1.0, -0.5, 0.0, 0.5, 1.0 ],
      "screw_direction": "RIGHT_HANDED",
      "ct": [
        [0.09802344228991167, 0.4988215184225952, 0.8231778824097602, 0.6578735041816796, -0.030677575975738707, -0.18100516740735045, -0.48816928539246107, -0.5422513249948423, -0.241955827686086, 0.09760901400000001],
        [ 0.02180769973655439, 0.6186028062460024, 1.1077294146069343, 0.9309423820559046, 0.11061832508836672, -0.27180694740557126, -0.6533257797843176, -0.7060443450793658, -0.2945433086645531, 0.022564886255851284],
        [ -0.000816229983420568, 0.5549690270137281, 1.1296393536152063, 1.0118939720603464, 0.2773688034374328, -0.3159968292076525, -1.0262724516816766, -1.1171690884802659, -0.5106919252572178, 0.0008162299834205433],
        [ -0.02820610781981364, 0.4006411665291089, 0.8950633072029262, 0.8124318138730758, 0.3060672901735377, -0.1790999500696448, -1.2017792679629602, -1.3874039779622673, -0.7185487731319289, -0.0272596246706925],
        [ -0.1626816899999993, 0.44866883773853117, 0.8944023203183769, 0.8032479305281881, 0.275724013778679, 0.017939934846272, -1.1427245405281872, -1.3839567584729409, -0.7788844482089198, -0.16337240381651863]],
      "cq": [
        [ -0.0114106274032369, -0.05096469487178818, -0.07146106086730822, -0.06198631583205713, 0.010279686325922563, 0.03675019835103964, 0.07774793189095229, 0.0664279441760814, 0.026284701001934016, -0.011165988600000013],
        [ -0.003522504562149637, -0.037319865766268494, -0.05041659824975429, -0.03801876017127842, -0.002641774188784713, 0.018157851375276573, 0.05486183456224583, 0.0612421985324825, 0.02205770495245041, -0.002775729928764877],
        [ 0.0022641186088044707, -0.004736600934521865, 0.012580274323343773, 0.008846389031002174, 0.0009727528373047617, 0.0010769429488141113, 0.009630382758339157, 0.010144557233001053, -0.006136809490287035, 0.0022641186088044776],
        [ -0.00346966241095603, 0.03117658791007869, 0.07708674165515654, 0.0675686590540254, 0.02166644052249731, -0.004430378368613751, -0.04909244207457484, -0.06520860571480078, -0.04369741490843816, -0.004403130702686983],
        [ -0.018609980999999907, 0.05512484306005881, 0.10925670286110455, 0.12550483840789847, 0.055566309226283736, 0.015411367752487103, -0.10401960640789848, -0.12289300851577122, -0.08100901238416293, -0.019017712338728045]
      ]
    })";


  json CPP_json = {{"propeller", {
                                     {"reference", "test"},
                                     {"type", "CPP"},
                                     {"open_water_table", json::parse(open_water_data_table)}
                                 }}
  };

  std::string filename = "temp.json";
  std::ofstream file;
  file.open(filename, std::ios::trunc);
  file << CPP_json.dump(2);
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
  auto frydom_CPP = make_controllable_pitch_propeller("CPP", body, node->GetNodePositionInBody(NWU), filename,
                                                      NWU);
  frydom_CPP->SetDiameter(diameter_m);
  frydom_CPP->SetScrewDirection(SCREW_DIRECTION::RIGHT_HANDED);
  frydom_CPP->SetRotationalVelocity(rpm, RPM);
  frydom_CPP->SetPitchRatio(pitch_ratio);
  frydom_CPP->SetStraightRunWakeFraction(hull_wake_fraction_0);
  frydom_CPP->SetThrustDeductionFactor(thrust_deduction_factor_0);

  acme::PropellerParams params(diameter_m, hull_wake_fraction_0, thrust_deduction_factor_0,
                               acme::SCREW_DIRECTION::RIGHT_HANDED);

  auto acme_CPP = acme::CPP(params, open_water_data_table);

  system.Initialize();
  acme_CPP.Initialize();
  acme_CPP.Compute(rho, u_NWU, v_NWU, rpm, pitch_ratio);
  std::cout << "beta : " << frydom_CPP->ComputeAdvanceAngle() << std::endl;
  std::cout << "ACME Thrust :" << acme_CPP.GetThrust() << std::endl;
  std::cout << "ACME Torque :" << acme_CPP.GetTorque() << std::endl;
  std::cout << "ACME Power :" << acme_CPP.GetPower() << std::endl;
  std::cout << "ACME Efficiency :" << acme_CPP.GetPropellerEfficiency() << std::endl;

  std::cout << "FRyDoM Thrust :" << frydom_CPP->GetThrust() << std::endl;
  std::cout << "FRyDoM Torque :" << frydom_CPP->GetTorque() << std::endl;
  std::cout << "FRyDoM Power :" << frydom_CPP->GetPower() << std::endl;

  return 0;

}

int test_rudder() {

  FrOffshoreSystem system("test_rudder");
  auto rho = 1025;
  system.GetEnvironment()->GetOcean()->SetDensity(rho);

  system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
  auto current = system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform();
  current->SetNorth(1.0, MS, COMEFROM);
//  current->Set(-20, 1, DEG, MS, NWU, COMEFROM);
  double rudder_angle_deg = 20;

  auto body = system.NewBody("body");
  body->SetFixedInWorld(true);
//  body->SetVelocityInBodyNoRotation(Velocity(-1.0, 0., 0.), NWU);

  auto node = body->NewNode("node");

  double hull_wake_fraction_0 = 0.;
  double height_m = 3;
  double chord_length_m = 3;
  double rudder_lateral_area_m2 = 15;
  double ramp_slope_degs = 4;

  std::vector<double> flow_incidence_on_main_rudder_deg = {-28, -26, -24.0, -22.0, -20.0, -18.0, -16.0, -14.0, -12.0,
                                                           -10.0, -8.0, -6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0,
                                                           12.0, 14.0, 16.0, 18.0, 20.0, 22.0, 24.0, 26.0, 28.0};
  std::vector<double> cd = {0.2778935134410858, 0.24202793836593628, 0.18181930482387543, 0.09083189815282822,
                            0.04477076604962349, 0.026631886139512062, 0.019129594787955284, 0.015245308168232441,
                            0.012423327192664146, 0.01028597541153431, 0.008432770147919655, 0.007111922837793827,
                            0.006130721885710955, 0.00546258594840765, 0.005242994520813227, 0.00546258594840765,
                            0.006130721885710955, 0.007111922837793827, 0.008432770147919655, 0.01028597541153431,
                            0.012423327192664146, 0.015245308168232441, 0.019129594787955284, 0.026631886139512062,
                            0.04477076604962349, 0.09083189815282822, 0.18181930482387543, 0.24202793836593628,
                            0.2778935134410858};
  std::vector<double> cl = {-1.1561044454574585, -1.1501551866531372,
                            -1.2516950368881226, -1.5970512628555298, -1.742898941040039, -1.7230695486068726,
                            -1.6346718072891235, -1.4956920146942139, -1.316851258277893, -1.1179311275482178,
                            -0.8795303702354431, -0.6680029630661011, -0.4484163820743561, -0.2255769968032837,
                            2.668157685548067e-05, 0.2255769968032837, 0.4484163820743561, 0.6680029630661011,
                            0.8795303702354431, 1.1179311275482178, 1.316851258277893, 1.4956920146942139,
                            1.6346718072891235, 1.7230695486068726, 1.742898941040039, 1.5970512628555298,
                            1.2516950368881226, 1.1501551866531372, 1.1561044454574585};
  std::vector<double> cn = {0.11684787273406982, 0.08721555024385452, 0.04257682338356972, -0.011721551418304443,
                            -0.03142339736223221, -0.03119639679789543, -0.0205577053129673, -0.009575597010552883,
                            -0.0033826581202447414, 0.00010325611947337165, -0.004063922446221113,
                            -0.0013044830411672592, -0.00022130433353595436, 0.00012940994929522276,
                            -3.3430785606469726e-06, -0.00012940994929522276, 0.00022130433353595436,
                            0.0013044830411672592, 0.004063922446221113, -0.00010325611947337165,
                            0.0033826581202447414, 0.009575597010552883, 0.0205577053129673, 0.03119639679789543,
                            0.03142339736223221, 0.011721551418304443, -0.04257682338356972, -0.08721555024385452,
                            -0.11684787273406982};

  std::stringstream ss;
  ss << R"({"flow_incidence_on_main_rudder_deg": )"
     << str(flow_incidence_on_main_rudder_deg.begin(), flow_incidence_on_main_rudder_deg.end())
     << R"(, "Cd": )" << str(cd.begin(), cd.end())
     << R"(, "Cl": )" << str(cl.begin(), cl.end())
     << R"(, "Cn": )" << str(cn.begin(), cn.end())
     << R"(, "frame_convention": "NWU", "direction_convention": "COMEFROM")"
     << "}";
//  std::cout<<ss.str()<<std::endl;


  Velocity rudder_relative_velocity = -system.GetEnvironment()->GetRelativeVelocityInFrame(node->GetFrameInWorld(),
                                                                                           node->GetVelocityInWorld(
                                                                                               NWU),
                                                                                           WATER,
                                                                                           NWU);
  auto u_NWU = rudder_relative_velocity.GetVx();
  auto v_NWU = rudder_relative_velocity.GetVy();
  std::cout << "u_NWU :" << u_NWU << std::endl;
  std::cout << "v_NWU :" << v_NWU << std::endl;

  // acme rudder model
  acme::RudderParams params;
  params.m_hull_wake_fraction_0 = hull_wake_fraction_0;
  params.m_lateral_area_m2 = rudder_lateral_area_m2;
  params.m_chord_m = chord_length_m;
  params.m_height_m = height_m;
  params.m_flap_slope = 0.; //NA

  auto acme_rudder = make_ACME_rudder("ACME_rudder", node, params, ss.str(), acme::E_SIMPLE_RUDDER);
  acme_rudder->SetRudderRampSlope(ramp_slope_degs);
  acme_rudder->SetRudderCommandAngle(rudder_angle_deg, DEG);
//    auto acme_rudder = acme::SimpleRudderModel(params, ss.str());
//    acme_rudder.Initialize();
//    acme_rudder.Compute(rho, u_NWU, v_NWU, rudder_angle_deg);
//  std::cout << "acme Fx = " << acme_rudder.GetFx() << std::endl;
//  std::cout << "acme Fy = " << acme_rudder.GetFy() << std::endl;
//  std::cout << "acme Mz = " << acme_rudder.GetMz() << std::endl;




  { // Old FRyDoM model
    json rudder_json = {{"rudder", {
                                       {"reference", "test"},
                                       {"load_coefficients", json::parse(ss.str())}
                                   }}
    };

    std::string filename = "temp.json";
    std::ofstream file;
    file.open(filename, std::ios::trunc);
    file << rudder_json.dump(2);
    file.close();

    auto frydom_rudder = make_rudder_force("frydom_rudder", body, node, filename);

    frydom_rudder->SetStraightRunWakeFraction(hull_wake_fraction_0);
    frydom_rudder->SetProjectedLateralArea(rudder_lateral_area_m2);
    frydom_rudder->SetHeight(height_m);
    frydom_rudder->SetRampSlope(ramp_slope_degs, DEGS);
    frydom_rudder->SetRootChord(chord_length_m);
    frydom_rudder->ActivateHullRudderInteraction(true);
    frydom_rudder->ActivateHullInfluenceOnTransverseVelocity(false);

    frydom_rudder->SetRudderAngle(rudder_angle_deg, DEG);

    system.Initialize();

    system.AdvanceTo(30);

    auto forceInBody = acme_rudder->GetForceInBody(NWU);

    std::cout << "rudder angle = " << acme_rudder->GetRudderAngle(DEG) << std::endl;
    std::cout << "acme Fx = " << forceInBody.GetFx() << std::endl;
    std::cout << "acme Fy = " << forceInBody.GetFy() << std::endl;
    std::cout << "acme Mz = "
              << acme_rudder->GetTorqueInBodyAtPointInBody(node->GetNodePositionInBody(NWU), NWU).GetMz()
              << std::endl;

    forceInBody = frydom_rudder->GetForceInBody(NWU);

    std::cout << "rudder angle = " << frydom_rudder->GetRudderAngle(DEG) << std::endl;
    std::cout << "FRyDoM Fx = " << forceInBody.GetFx() << std::endl;
    std::cout << "FRyDoM Fy = " << forceInBody.GetFy() << std::endl;
    std::cout << "FRyDoM Mz = "
              << frydom_rudder->GetTorqueInBodyAtPointInBody(node->GetNodePositionInBody(NWU), NWU).GetMz()
              << std::endl;
  }

  return 0.;
}

int main() {
//  return test_FPP1Q();
  return test_rudder();
}