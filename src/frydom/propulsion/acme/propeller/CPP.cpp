//
// Created by frongere on 04/08/2021.
//

#include "CPP.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace acme {

  CPP::CPP(const PropellerParams &params, const std::string &ct_cq_json_string) :
      FPP4Q(params, ct_cq_json_string) {
    m_type = PropellerModelType::E_CPP;  // Overrides the type E_FPP4Q
  }

  void CPP::GetCtCq(const double &gamma,
                    const double &pitch_ratio,
                    double &ct,
                    double &cq) const {
    ct = m_ct_ct_coeffs.Eval("ct", gamma, pitch_ratio);
    cq = m_ct_ct_coeffs.Eval("cq", gamma, pitch_ratio);
  }

  void CPP::ParsePropellerPerformanceCurveJsonString() {

    std::vector<double> beta, pitch_ratio, ct, cq;
    ParseCPPJsonString(m_temp_perf_data_json_string, beta, pitch_ratio, ct, cq);
    m_ct_ct_coeffs.SetX(beta);
    m_ct_ct_coeffs.SetY(pitch_ratio);
    m_ct_ct_coeffs.AddData("ct", ct);
    m_ct_ct_coeffs.AddData("cq", cq);

  }

  void ParseCPPJsonString(const std::string &json_string, std::vector<double> &beta,
                          std::vector<double> &pitch_ratio, std::vector<double> &ct,
                          std::vector<double> &cq) {

    auto jnode = json::parse(json_string);

    beta = jnode["beta_deg"].get<std::vector<double>>();
    pitch_ratio = jnode["p_d"].get<std::vector<double>>();
    auto ct_tmp = jnode["ct"].get<std::vector<std::vector<double>>>();
    auto cq_tmp = jnode["cq"].get<std::vector<std::vector<double>>>();

    for (auto &b : beta) b *= DEG2RAD;

    //FIXME
//    // Only one
//    if (screw_direction == "LEFT_HANDED") {
//      for (auto &c : kq) {
//        c = -c;
//      }
//    } else if (screw_direction == "RIGHT_HANDED") {
//      // Nothing
//    } else {
//      std::cerr << "Unknown screw direction " << screw_direction << std::endl;
//      exit(EXIT_FAILURE);
//    }

    // Transform ct to fit in AddData
    Eigen::MatrixXd mat(pitch_ratio.size(), beta.size());
    for (int i = 0; i < pitch_ratio.size(); i++) {
      mat.row(i) = Eigen::VectorXd::Map(&ct_tmp[i][0], ct_tmp[i].size());
    }
    ct = {mat.data(), mat.data() + mat.rows() * mat.cols()};

    // Transform cq to fit in AddData
    for (int i = 0; i < pitch_ratio.size(); i++) {
      mat.row(i) = Eigen::VectorXd::Map(&cq_tmp[i][0], cq_tmp[i].size());
    }
    cq = {mat.data(), mat.data() + mat.rows() * mat.cols()};
  }
}  // end namespace acme