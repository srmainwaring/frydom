//
// Created by frongere on 04/08/2021.
//

#include "CPP.h"

namespace acme {

  CPP::CPP(const ThrusterParams &params, const std::string &ct_cq_json_string) :
      FPP4Q(params, ct_cq_json_string) {
    m_type = ThrusterModelType::E_CPP;  // Overrides the type E_FPP4Q
  }

  void CPP::GetCtCq(const double &gamma,
                    const double &pitch_ratio,
                    double &ct,
                    double &cq) const {
    ct = m_ct_ct_coeffs.Eval("ct", gamma, pitch_ratio);
    cq = m_ct_ct_coeffs.Eval("cq", gamma, pitch_ratio);
  }

  void CPP::ParsePropellerPerformanceCurveJsonString() {

  }

}  // end namespace acme