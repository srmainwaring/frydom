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
    CPP(const ThrusterParams &params, const std::string &ct_cq_json_string);

   private:

    inline void GetCtCq(const double &gamma,
                        const double &pitch_ratio,
                        double &ct,
                        double &cq) const override;

    void ParsePropellerPerformanceCurveJsonString() override;

   private:
    mathutils::LookupTable2d<double> m_ct_ct_coeffs;

  };

}  // end namespace acme

#endif //ACME_CPP_H
