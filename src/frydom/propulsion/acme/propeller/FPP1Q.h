//
// Created by frongere on 04/08/2021.
//

#ifndef ACME_FPP1Q_H
#define ACME_FPP1Q_H

#include <string>

#include "MathUtils/LookupTable1D.h"

#include "PropellerBaseModel.h"

namespace acme {

  /// First quadrant model for Fixed Pitch Propeller
  class FPP1Q : public PropellerBaseModel {

   public:
    FPP1Q(const PropellerParams &params, const std::string &kt_kq_json_string);

    void Compute(const double &water_density,
                 const double &u_NWU,
                 const double &v_NWU,
                 const double &rpm,
                 const double &pitch_ratio) const override; // pitch ratio not used in this model, may be any value

   private:

    void GetKtKq(const double &J, double &kt, double &kq) const;

    void ParsePropellerPerformanceCurveJsonString() override;

   private:
    mathutils::LookupTable1D<double, double> m_kt_kq_coeffs;

  };

}  // end namespace acme

#endif //ACME_FPP1Q_H
