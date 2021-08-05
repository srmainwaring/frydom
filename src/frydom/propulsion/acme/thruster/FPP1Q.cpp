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
      ThrusterBaseModel(params) {
    LoadFPP1Q_params(kt_kq_json_string);
  }

  double
  FPP1Q::ComputeThrust(const double &water_density, const double &advance_velocity_ms, const double &rpm) const {
    // FIXME: Attention: advance_velocity n'est pas la bonne mesure !!!

    double J = ComputeAdvanceRatio(advance_velocity_ms, rpm);
    return water_density * std::pow(m_params.m_diameter_m, 4) * kt(J) * rpm * rpm;
  }

  double
  FPP1Q::ComputeShaftTorque(const double &water_density, const double &advance_velocity_ms, const double &rpm) const {
    double J = ComputeAdvanceRatio(advance_velocity_ms, rpm);
    return water_density * std::pow(m_params.m_diameter_m, 5) * kq(J) * rpm * rpm * GetScrewDirectionSign();
  }

  void
  FPP1Q::ComputeThrustAndTorque(const double &water_density,
                                const double &advance_velocity_ms,
                                const double &rpm,
                                double &thrust,
                                double &torque) const {
    thrust = ComputeThrust(water_density, advance_velocity_ms, rpm);
    torque = ComputeShaftTorque(water_density, advance_velocity_ms, rpm);
  }

  double FPP1Q::ComputeAdvanceRatio(const double &advance_velocity_ms, const double &rpm) const {
    return 60. * advance_velocity_ms / (rpm * m_params.m_diameter_m);
  }

  void FPP1Q::LoadFPP1Q_params(const std::string &kt_kq_json_string) {
    auto jnode = json::parse(kt_kq_json_string);

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