//
// Created by frongere on 04/08/2021.
//

#include "PropellerBaseModel.h"

#include "MathUtils/Angles.h"

namespace acme {

  PropellerBaseModel::PropellerBaseModel(const PropellerParams params,
                                         const std::string &perf_data_json_string,
                                         PropellerModelType type) :
      m_params(params),
      m_temp_perf_data_json_string(perf_data_json_string),
      m_is_initialized(false),
      m_type(type),
      m_ku(1.),
      c_sidewash_angle_rad(0.),
      c_thrust_N(0.),
      c_torque_Nm(0.),
      c_efficiency(0.),
      c_power_W(0.) {
  }

  void PropellerBaseModel::Initialize() {
    ParsePropellerPerformanceCurveJsonString();
    m_temp_perf_data_json_string.clear();

    if (m_params.m_use_advance_velocity_correction_factor) ComputeAdvanceVelocityCorrectionFactor();

    m_is_initialized = true;
  }

  PropellerModelType PropellerBaseModel::GetThrusterModelType() const {
    return m_type;
  }

  const PropellerParams &PropellerBaseModel::GetParameters() const {
    return m_params;
  }

  double PropellerBaseModel::GetAdvanceVelocity() const {
    return c_uPA;
  }

  double PropellerBaseModel::GetThrust() const {
    return c_thrust_N;
  }

  double PropellerBaseModel::GetTorque() const {
    return c_torque_Nm;
  }

  double PropellerBaseModel::GetPropellerEfficiency() const {
    return c_efficiency;
  }

  double PropellerBaseModel::GetPower() const {
    return c_power_W;
  }

  double PropellerBaseModel::GetPropellerAdvanceVelocity(const double &u_NWU,
                                                         const double &v_NWU) const {
    // sidewash angle
    c_sidewash_angle_rad = mathutils::Normalize__PI_PI(std::atan2(v_NWU, u_NWU));

    // estimated wake_fraction taken into account the sidewash angle (0 when the absolute value of the sidewash
    // angle exceeds 90°)
    double wp = (std::abs(c_sidewash_angle_rad) > MU_PI_2) ? 0. :
                m_params.m_hull_wake_fraction_0 * std::exp(-4. * c_sidewash_angle_rad * c_sidewash_angle_rad);

    // Propeller advance velocity
    c_uPA = m_ku * u_NWU * (1 - wp);
    return c_uPA;
  }

  void PropellerBaseModel::ComputeAdvanceVelocityCorrectionFactor() {
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


  PropellerParams::PropellerParams(double diameter_m, double hull_wake_fraction_0,
                                   double thrust_deduction_factor_0, SCREW_DIRECTION sd) :
      m_diameter_m(diameter_m), m_hull_wake_fraction_0(hull_wake_fraction_0),
      m_thrust_deduction_factor_0(thrust_deduction_factor_0),
      m_screw_direction(sd) {

  }

  PropellerParams::PropellerParams() : m_diameter_m(0.), m_hull_wake_fraction_0(0.),
                                       m_thrust_deduction_factor_0(0.),
                                       m_screw_direction(RIGHT_HANDED) {

  }
}  // end namespace acme