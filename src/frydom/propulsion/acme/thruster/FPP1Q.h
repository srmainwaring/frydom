//
// Created by frongere on 04/08/2021.
//

#ifndef ACME_FPP1Q_H
#define ACME_FPP1Q_H

#include <string>
#include "MathUtils/LookupTable1D.h"

#include "ThrusterBaseModel.h"

namespace acme {

  /// First quadrant model for Fixed Pitch Propeller
  class FPP1Q : public ThrusterBaseModel {

   public:
    FPP1Q(const ThrusterBaseParams &params, const std::string &kt_kq_json_string);

    void Compute(const double &water_density,
                 const double &u_NWU,
                 const double &v_NWU,
                 const double &rpm,
                 const double &pitch_ratio) const override; // pitch ratio not used in this model, may be any value

//    void Initialize() override;

   private:

    inline double kt(const double &J) const;

    inline double kq(const double &J) const;

    void ParsePropellerPerformanceCurveJsonString() override;

   private:
    mathutils::LookupTable1D<double> m_kt_kq_coeffs;

  };

}  // end namespace acme

#endif //ACME_FPP1Q_H
