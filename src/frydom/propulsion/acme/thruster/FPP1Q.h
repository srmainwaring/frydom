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

    double ComputeThrust(const double &water_density,
                         const double &advance_velocity_ms,
                         const double &rpm) const override;

    double ComputeShaftTorque(const double &water_density,
                              const double &advance_velocity_ms,
                              const double &rpm) const override;

    void ComputeThrustAndTorque(const double &water_density,
                                const double &advance_velocity_ms,
                                const double &rpm,
                                double &thrust,
                                double &torque) const override;

   private:
    inline double ComputeAdvanceRatio(const double &advance_velocity_ms, const double &rpm) const;

    inline double kt(const double &J) const;
    inline double kq(const double &J) const;

    void LoadFPP1Q_params(const std::string &kt_kq_json_string);

   private:
    mathutils::LookupTable1D<double> m_kt_kq_coeffs;
//    bool m_is_right_handed;


  };

}  // end namespace acme

#endif //ACME_FPP1Q_H
