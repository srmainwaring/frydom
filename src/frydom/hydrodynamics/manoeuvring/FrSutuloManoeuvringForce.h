//
// Created by lletourn on 04/06/2021.
//

#ifndef FRYDOM_FRSUTULOMANOEUVRINGFORCE_H
#define FRYDOM_FRSUTULOMANOEUVRINGFORCE_H

#include <utility>

#include "frydom/core/force/FrForce.h"

namespace frydom {

  // forward declaration
  class FrHullResistance;
  class FrHullResistanceForce;

  struct FrSutuloManCoefficients {
    double m_cm = 0.;
    double m_mu22 = 0.;
    double m_cy0 = 0;
    double m_cy1 = 0;
    double m_cy2 = 0;
    double m_cy3 = 0;
    double m_cy4 = 0;
    double m_cy5 = 0;
    double m_cy6 = 0;
    double m_cy7 = 0;
    double m_cy8 = 0;
    double m_cn0 = 0;
    double m_cn1 = 0;
    double m_cn2 = 0;
    double m_cn3 = 0;
    double m_cn4 = 0;
    double m_cn5 = 0;
    double m_cn6 = 0;
    double m_cn7 = 0;
    double m_cn8 = 0;
    double m_cn9 = 0;
  };

  class FrSutuloManoeuvringForce : public FrForce {

   public:

    FrSutuloManoeuvringForce(const std::string& name, FrBody* body,
                             const FrSutuloManCoefficients& coeffs,
                             const std::shared_ptr<FrHullResistanceForce>& hullResistanceForce,
                             double draft_m, double lpp_m);

    void Initialize() override;

    double GetUMin() const;

    double GetUMax() const;

   private:

    static double ComputeShipDriftAngle(const double &u, const double &v);

    static double ComputeYawAdimVelocity(const double &u, const double &v, const double &r, const double &L);

    void Compute(double time) override;

    double Rh(double u) const;

    void DefineLogMessages() override;

   private:
    std::shared_ptr<FrHullResistanceForce> m_hullResistanceForce;

    double m_Cm, m_mu22;
    double m_cy0, m_cy1, m_cy2, m_cy3, m_cy4, m_cy5, m_cy6, m_cy7, m_cy8;
    double m_cn0, m_cn1, m_cn2, m_cn3, m_cn4, m_cn5, m_cn6, m_cn7, m_cn8, m_cn9;

    double c_beta; // drift angle
    double c_rpp; // yaw adim velocioty

    double c_Xpp, c_Xpp0, c_Xpp1;
    double c_Ypp, c_Ypp0, c_Ypp1, c_Ypp2, c_Ypp3, c_Ypp4, c_Ypp5, c_Ypp6, c_Ypp7, c_Ypp8;
    double c_Npp, c_Npp0, c_Npp1, c_Npp2, c_Npp3, c_Npp4, c_Npp5, c_Npp6, c_Npp7, c_Npp8, c_Npp9;

    double m_lpp;
    double m_draft;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  std::shared_ptr<FrSutuloManoeuvringForce>
      make_sutulo_manoeuvring_model(const std::string& name, std::shared_ptr<FrBody> body, const FrSutuloManCoefficients& coeffs,
                                    const std::shared_ptr<FrHullResistanceForce>& hullResistanceForce,
                                    double draft_m, double lpp_m);


}// end namespace frydom

#endif //FRYDOM_FRSUTULOMANOEUVRINGFORCE_H
