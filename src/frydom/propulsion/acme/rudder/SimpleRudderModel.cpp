//
// Created by frongere on 09/08/2021.
//

#include "SimpleRudderModel.h"

#include <iostream>
#include <vector>
#include <nlohmann/json.hpp>
#include "MathUtils/Unit.h"

namespace acme {

  SimpleRudderModel::SimpleRudderModel(const RudderParams params, const std::string &perf_data_json_string) :
      m_params(params),
      m_temp_perf_data_json_string(perf_data_json_string),
      m_is_initialized(false) {

  }

  void SimpleRudderModel::Initialize() {
    ParseRudderPerformanceCurveJsonString();
    // TODO: initialiser ici le cache des quantites optionnelles


    m_is_initialized = true;
  }

  void SimpleRudderModel::Compute(const double &water_density,
                                  const double &u_NWU,
                                  const double &v_NWU,
                                  const double &rudder_angle_deg) const {

    if (!m_is_initialized) {
      std::cerr << "Propulsion model MUST be initialized before being used." << std::endl;
      exit(EXIT_FAILURE);
    }

    double sidewash_angle_0 = std::atan2(v_NWU, u_NWU); // ou drift_angle_0 pour calculer le wake fraction

    // Estimated wake fraction
    double wr = m_params.m_hull_wake_fraction_0 * std::exp(-4. * sidewash_angle_0 * sidewash_angle_0);

    double uRA = u_NWU * (1 - wr);
    double vRA = v_NWU;

//    if (m_params.m_has_hull_influence_transverse_velocity) {
//
//      double kappa = 1.; // TODO: implementer le calcul
//      vRA *= kappa;
//    }

    double V2 = uRA * uRA + vRA * vRA;

    // Drift angle
    double beta_R = std::atan2(vRA, uRA);

    // Attack angle
    double rudder_angle_rad = rudder_angle_deg * MU_PI_180;
    c_alpha_R_rad = rudder_angle_rad - beta_R;

    // Get coefficients
    double cl, cd, cn;
    GetClCdCn(c_alpha_R_rad, rudder_angle_rad, cl, cd, cn);

    // Forces in flow frame
    c_lift_N = 0.5 * water_density * cl * m_params.m_lateral_area_m2 * V2; // FIXME: la prise en compte du signe de alpha se fait comment ? dans les tables ?
    c_drag_N = 0.5 * water_density * cd * m_params.m_lateral_area_m2 * V2; // FIXME: les tables prevoient des coeffs cd negatifs ?
    c_torque_Nm = 0.5 * water_density * cn * m_params.m_lateral_area_m2 * m_params.m_chord_m * V2; // FIXME: la prise en compte du signe de alpha se fait comment ? dans les tables ?

    // Forces in rudder frame
    double Cbeta = std::cos(beta_R);
    double Sbeta = std::sin(beta_R);

    c_fx_N = Cbeta * c_drag_N - Sbeta * c_lift_N;
    c_fy_N = Sbeta * c_drag_N + Cbeta * c_lift_N;

  }

  void SimpleRudderModel::GetClCdCn(const double &attack_angle_rad,
                                    const double &rudder_angle_rad,
                                    double &cl,
                                    double &cd,
                                    double &cn) const {

    cl = m_cl_cd_cn_coeffs.Eval("cl", attack_angle_rad);
    cd = m_cl_cd_cn_coeffs.Eval("cd", attack_angle_rad);
    cn = m_cl_cd_cn_coeffs.Eval("cn", attack_angle_rad);
  }


  void SimpleRudderModel::ParseRudderPerformanceCurveJsonString() {
    // TODO
  }

}  // end namespace acme