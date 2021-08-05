//
// Created by frongere on 04/08/2021.
//

#include "ThrusterBaseModel.h"

namespace acme {

  ThrusterBaseModel::ThrusterBaseModel(const ThrusterBaseParams params, const std::string &perf_data_json_string) :
      m_params(params),
      m_perf_data_json_string(perf_data_json_string),
      m_ku(1.),
      m_is_initialized(false),
      c_sidewash_angle(0.),
      c_thrust(0.),
      c_torque(0.),
      c_efficiency(0.),
      c_power(0.) {
  }

  void ThrusterBaseModel::Initialize() {
    ParsePropellerPerformanceCurveJsonString();
    if (m_params.m_use_advance_velocity_correction_factor) ComputeAdvanceVelocityCorrectionFactor();
    m_is_initialized = true;
  }

  double acme::ThrusterBaseModel::GetThrust() const {
    return c_thrust;
  }

  double ThrusterBaseModel::GetTorque() const {
    return c_torque;
  }

  double ThrusterBaseModel::GetPropellerEfficiency() const {
    return c_efficiency;
  }

  double ThrusterBaseModel::GetPower() const {
    return c_power;
  }

  double ThrusterBaseModel::GetPropellerAdvanceVelocity(const double &u_NWU,
                                                        const double &v_NWU) const {
    // sidewash angle
    c_sidewash_angle = std::atan2(v_NWU, u_NWU);

    // estimated wake_fraction taken into account the sidewash angle (0 when the absolute value of the sidewash
    // angle exeeds 90°)
    double wp = (std::abs(c_sidewash_angle) > MU_PI_2) ? 0. :
                m_params.m_hull_wake_fraction_0 * std::exp(-4. * c_sidewash_angle * c_sidewash_angle);

    // Propeller advance velocity
    return m_ku * u_NWU * (1 - wp);
  }

  void ThrusterBaseModel::ComputeAdvanceVelocityCorrectionFactor() {
    /*
     * Jopt is the maximum efficiency advance ratio for the used propeller model
     */
    double Jopt = 1.; // TODO par recherche d'extremum de eta0 = J * kt / (2pi *kq)

    // Propeller design rps
    double nd = m_params.m_propeller_design_rpm / 60.;

    // Design advance ratio
    double Jd = (m_params.m_vessel_design_speed_ms * (1 - m_params.m_hull_wake_fraction_0)) /
                (nd * m_params.m_diameter_m);

    m_ku = Jopt / Jd;

  }


}  // end namespace acme