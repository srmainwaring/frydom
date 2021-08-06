//
// Created by frongere on 04/08/2021.
//

#include "CPP.h"

namespace acme {

  CPP::CPP(const ThrusterBaseParams &params, const std::string &ct_cq_json_string) :
      FPP4Q(params, ct_cq_json_string) {

  }

//  void CPP::Initialize() {
//    FPP4Q::Initialize();
//  }

  double CPP::ct(const double &gamma, const double &pitch_ratio) const {
    return m_ct_ct_coeffs.Eval("ct", gamma, pitch_ratio);
  }

  double CPP::cq(const double &gamma, const double &pitch_ratio) const {
    return m_ct_ct_coeffs.Eval("cq", gamma, pitch_ratio);
  }

  void CPP::ParsePropellerPerformanceCurveJsonString() {

  }

}  // end namespace acme