//
// Created by frongere on 12/08/2021.
//

#include <fstream>
#include <filesystem>
#include "acme/acme.h"
#include "nlohmann/json.hpp"

using namespace acme;
using json = nlohmann::json;
namespace fs = std::filesystem;

int main() {

  /**
   * Propeller definition
   */

  // Parameters

  PropellerParams propeller_params;
  propeller_params.m_diameter_m = 5.;
  propeller_params.m_screw_direction = RIGHT_HANDED;

  propeller_params.m_hull_wake_fraction_0 = 0.3;
  propeller_params.m_thrust_deduction_factor_0 = 0.1;

  // TODO: Donnees a ajouter dans le json
  propeller_params.m_use_advance_velocity_correction_factor = false;
  propeller_params.m_propeller_design_rpm;
  propeller_params.m_vessel_design_speed_ms;

  propeller_params.m_thrust_coefficient_correction = 0.;
  propeller_params.m_torque_coefficient_correction = 0.;


  // Perf data file
  std::string ktkq_file = "../../data/KtKq.json";

  if (!fs::exists(ktkq_file)) {
    std::cerr << "JSON file " << ktkq_file << " not found" << std::endl;
    exit(EXIT_FAILURE);
  }

  std::ifstream ifs(ktkq_file);
  json ktkq_node = json::parse(ifs);

  ktkq_node = ktkq_node["KTKQPropellerTable"];

  std::string ktkq_json_string = ktkq_node.dump();


  /**
   * Rudder definition
   */

  // Parameters
  RudderParams rudder_params;
  // TODO: remplir


  // Perf data file
  std::string clcdcn_json_string("");
  // TODO: remplir

  /**
   * Propeller rudder construction
   */

  auto propeller_rudder = build_pr(E_FPP1Q, propeller_params, ktkq_json_string,
                                   E_SIMPLE_RUDDER, rudder_params, clcdcn_json_string);




//  auto prop = FPP1Q(propeller_params, ktkq_json_string);
//  prop.Initialize();
//
//
//
//  prop.Compute(1025, 16 * MU_KNOT, 1*MU_KNOT, 80, 0);
//
//  std::cout << "Thrust: " << prop.GetThrust() * 1e-3 << " kN" << std::endl;
//  std::cout << "Torque: " << prop.GetTorque() * 1e-3 << " kN.m" << std::endl;
//  std::cout << "Efficiency: " << prop.GetPropellerEfficiency() << std::endl;
//  std::cout << "Power: " << prop.GetPower() * 1e-3 << " kW" << std::endl;


  return 0;
}