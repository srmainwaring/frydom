//
// Created by frongere on 04/08/2021.
//

#include "FPP1Q.h"

#include <iostream>
#include <vector>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace acme {

  FPP1Q::FPP1Q(const ThrusterBaseParams &params, const std::string &kt_kq_json_string) :
      ThrusterBaseModel(params, kt_kq_json_string) {
  }

  void FPP1Q::Initialize() {
    ThrusterBaseModel::Initialize();
  }

  void FPP1Q::Compute(const double &water_density,
                      const double &u_NWU,
                      const double &v_NWU,
                      const double &rpm,
                      const double &pitch_ratio) const {

    if (!m_is_initialized) {
      std::cerr << "Propulsion model MUST be initialized before being used." << std::endl;
      exit(EXIT_FAILURE);
    }

    // Propeller advance velocity
    double uPA = GetPropellerAdvanceVelocity(u_NWU, v_NWU);

    // propeller rotation frequency in rps
    double n = rpm / 60.;

    // Advance ratio
    double J = uPA / (n * m_params.m_diameter_m);

    // Propeller Thrust
    double n2 = n * n;
    double _kt = kt(J) + m_params.m_thrust_corr;
    double propeller_thrust = water_density * n2 * std::pow(m_params.m_diameter_m, 4) * _kt;

    // Effective propeller thrust
    c_thrust = propeller_thrust * (1 - m_params.m_thrust_deduction_factor_0);

    // Torque
    double _kq = kq(J) + m_params.m_torque_corr;
    c_torque = water_density * n2 * std::pow(m_params.m_diameter_m, 5) * _kq * GetScrewDirectionSign();

    // Efficiency
    c_efficiency = J * _kt / (MU_2PI * _kq);

    // Power
    c_power = n * c_torque;

  }

  void FPP1Q::ParsePropellerPerformanceCurveJsonString() {
    // TODO: terminer, pas fini !!!
    auto jnode = json::parse(m_perf_data_json_string);
    m_perf_data_json_string.clear();

    auto j = jnode["j"].get<std::vector<double>>();
    auto kt = jnode["kt"].get<std::vector<double>>();
    auto kq = jnode["kq"].get<std::vector<double>>();

    auto screw_direction = jnode["screw_direction"].get<std::string>();

    // Only one
    if (screw_direction == "LEFT_HANDED") {
      for (auto &c : kq) {
        c = -c;
      }
    } else if (screw_direction == "RIGHT_HANDED") {
      // Nothing
    } else {
      std::cerr << "Unknown screw direction " << screw_direction << std::endl;
      exit(EXIT_FAILURE);
    }

    m_kt_kq_coeffs.SetX(j);
    m_kt_kq_coeffs.AddY("kt", kt);
    m_kt_kq_coeffs.AddY("kq", kq);
  }

  double FPP1Q::kt(const double &J) const {
    return m_kt_kq_coeffs.Eval("kt", J);
  }

  double FPP1Q::kq(const double &J) const {
    return m_kt_kq_coeffs.Eval("kq", J);
  }


}  // end namespace acme