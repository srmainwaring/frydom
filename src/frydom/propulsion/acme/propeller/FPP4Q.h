//
// Created by frongere on 04/08/2021.
//

#ifndef ACME_FPP4Q_H
#define ACME_FPP4Q_H

#include <string>
#include "MathUtils/LookupTable1D.h"

#include "PropellerBaseModel.h"

namespace acme {

  /// Four Quadrant model for Fixed Pitch Propeller
  class FPP4Q : public PropellerBaseModel {

   public:
    FPP4Q(const PropellerParams &params, const std::string &ct_cq_json_string);

    void Compute(const double &water_density,
                 const double &u_NWU,
                 const double &v_NWU,
                 const double &rpm,
                 const double &pitch_ratio) const override; // pitch ratio not used in this model, may be any value

   private:

    virtual void GetCtCq(const double &gamma,
                         const double &pitch_ratio,
                         double &ct,
                         double &cq) const;

    void ParsePropellerPerformanceCurveJsonString() override;

   private:
    mathutils::LookupTable1D<double, double> m_ct_ct_coeffs;

  };

}  // end namespace acme

#endif //ACME_FPP4Q_H
