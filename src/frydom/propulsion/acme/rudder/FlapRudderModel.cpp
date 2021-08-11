//
// Created by frongere on 09/08/2021.
//

#include "FlapRudderModel.h"

namespace acme {


  FlapRudderModel::FlapRudderModel(const RudderParams params, const std::string &perf_data_json_string)
      : SimpleRudderModel(params, perf_data_json_string) {
    m_type = RudderModelType::E_FLAP_RUDDER;  // Overrides the E_SIMPLE_RUDDER
  }

  void FlapRudderModel::GetClCdCn(const double &attack_angle_rad,
                                  const double &rudder_angle_rad,
                                  double &cl,
                                  double &cd,
                                  double &cn) const {

    // Getting the flap angle from the rudder angle using the linear law (only linear law currently supported)
    double flap_angle_rad = m_params.m_flap_slope * rudder_angle_rad;

    cl = m_cl_cd_cn_coeffs.Eval("cl", attack_angle_rad, flap_angle_rad);
    cd = m_cl_cd_cn_coeffs.Eval("cd", attack_angle_rad, flap_angle_rad);
    cn = m_cl_cd_cn_coeffs.Eval("cn", attack_angle_rad, flap_angle_rad);

  }

  void FlapRudderModel::ParseRudderPerformanceCurveJsonString() {
    // TODO
  }


}  // end namespace acme