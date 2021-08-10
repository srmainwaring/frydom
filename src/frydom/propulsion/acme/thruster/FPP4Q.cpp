//
// Created by frongere on 04/08/2021.
//

#include "FPP4Q.h"

namespace acme {

  FPP4Q::FPP4Q(const acme::ThrusterParams &params, const std::string &ct_cq_json_string) :
      ThrusterBaseModel(params, ct_cq_json_string, ThrusterModelType::E_FPP4Q) {
  }

  void FPP4Q::Compute(const double &water_density,
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

    // tangential blade velocity at 0.7R
    double vp = 0.7 * MU_PI * n * m_params.m_diameter_m;

    // Effective blade advance angle
    double gamma = std::atan2(uPA, vp);

    // Effective total velocity squared
    double vB2 = uPA * uPA + vp * vp;

    // Propeller disk area
    double Ad = MU_PI * m_params.m_diameter_m * m_params.m_diameter_m / 4.;

    // Get Coefficients
    double ct, cq;
    GetCtCq(gamma, pitch_ratio, ct, cq);

    // Propeller Thrust
    double _ct = ct + m_params.m_thrust_coefficient_correction;
    double propeller_thrust = 0.5 * water_density * vB2 * Ad * _ct;

    // Effective propeller thrust
    c_thrust_N = propeller_thrust * (1 - m_params.m_thrust_deduction_factor_0);

    // Torque
    double _cq = cq + m_params.m_torque_coefficient_correction;
    c_torque_Nm = 0.5 * water_density * vB2 * Ad * m_params.m_diameter_m * _cq;

    // Efficiency
    if (n != 0.) {
      double J = uPA / (n * m_params.m_diameter_m);
      c_efficiency = J * _ct / (MU_2PI * _cq);
    } else {
      c_efficiency = 0.; // TODO: voir si on met 0 ou 1...
    }

    // Power
    c_power_W = n * c_torque_Nm;

  }

  void FPP4Q::GetCtCq(const double &gamma,
                      const double &pitch_ratio,
                      double &ct,
                      double &cq) const {

    ct = m_ct_ct_coeffs.Eval("ct", gamma);
    cq = m_ct_ct_coeffs.Eval("cq", gamma);
  }

  void FPP4Q::ParsePropellerPerformanceCurveJsonString() {
    // TODO
  }

}  // end namespace acme