//
// Created by frongere on 06/08/2021.
//

#include <fstream>
#include <filesystem>
#include "acme/acme.h"
#include "nlohmann/json.hpp"

using namespace acme;
using json = nlohmann::json;
namespace fs = std::filesystem;

int main() {

  PropellerParams params;
  params.m_diameter_m = 5.;
  params.m_screw_direction = RIGHT_HANDED;

  params.m_hull_wake_fraction_0 = 0.3;
  params.m_thrust_deduction_factor_0 = 0.1;

  // TODO: Donnees a ajouter dans le json
  params.m_use_advance_velocity_correction_factor = false;
  params.m_propeller_design_rpm;
  params.m_vessel_design_speed_ms;

  params.m_thrust_coefficient_correction = 0.;
  params.m_torque_coefficient_correction = 0.;


  std::string ktkq_file = "../../data/KtKq.json";

  if (!fs::exists(ktkq_file)) {
    std::cerr << "JSON file " << ktkq_file << " not found" << std::endl;
    exit(EXIT_FAILURE);
  }

  std::ifstream ifs(ktkq_file);
  json node = json::parse(ifs);

  node = node["KTKQPropellerTable"];

  std::string json_string = node.dump();

//  std::cout << json_string << std::endl;


  auto prop = FPP1Q(params, json_string);
  prop.Initialize();



  prop.Compute(1025, 16 * MU_KNOT, 1*MU_KNOT, 80, 0);

  std::cout << "Thrust: " << prop.GetThrust() * 1e-3 << " kN" << std::endl;
  std::cout << "Torque: " << prop.GetTorque() * 1e-3 << " kN.m" << std::endl;
  std::cout << "Efficiency: " << prop.GetPropellerEfficiency() << std::endl;
  std::cout << "Power: " << prop.GetPower() * 1e-3 << " kW" << std::endl;


  return 0;
}