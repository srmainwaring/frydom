//
// Created by frongere on 04/08/2021.
//

#ifndef ACME_CPP_H
#define ACME_CPP_H

#include <string>
#include "MathUtils/LookupTable2D.h"

#include "FPP4Q.h"

namespace acme {

  class CPP : public FPP4Q {

   public:
    CPP(const PropellerParams &params, const std::string &ct_cq_json_string);

   private:

    void GetCtCq(const double &gamma,
                 const double &pitch_ratio,
                 double &ct,
                 double &cq) const override;

    void ParsePropellerPerformanceCurveJsonString() override;

   private:
    mathutils::LookupTable2d<double> m_ct_ct_coeffs;

  };

  void ParseCPPJsonString(const std::string &json_string,
                          std::vector<double> &beta,
                          std::vector<double> &pitch_ratio,
                          std::vector<double> &ct,
                          std::vector<double> &cq);

}  // end namespace acme

#endif //ACME_CPP_H
