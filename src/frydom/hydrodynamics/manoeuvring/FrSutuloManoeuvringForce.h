//
// Created by lletourn on 04/06/2021.
//

#ifndef FRYDOM_FRSUTULOMANOEUVRINGFORCE_H
#define FRYDOM_FRSUTULOMANOEUVRINGFORCE_H

#include <utility>

#include "frydom/core/force/FrForce.h"
#include "MathUtils/Interp1d.h"

namespace frydom {

  class FrSutuloManoeuvringForce : public FrForce {

   public:

    FrSutuloManoeuvringForce(const std::string &name, FrBody *body, const std::string &file);

    void Initialize() override;

    double GetUMin() const { return m_hullResistance.GetXmin(); }

    double GetUMax() const { return m_hullResistance.GetXmax(); }

   private:

    static double ComputeShipDriftAngle(const double &u, const double &v);

    static double ComputeYawAdimVelocity(const double &u, const double &v, const double &r, const double &L);

    void LoadResistanceCurve(const std::string &filepath);

    void LoadManoeuvringData(const std::string &filepath);

    void Compute(double time) override;

    double Rh(double u) const;

    void DefineLogMessages() override;

   private:

    std::string c_filepath; ///< path to the JSON file containing the manoeuvring data

    mathutils::Interp1dLinear<double, double> m_hullResistance;
    double m_cy0, m_cy1, m_cy2, m_cy3, m_cy4, m_cy5, m_cy6, m_cy7, m_cy8;
    double m_cn0, m_cn1, m_cn2, m_cn3, m_cn4, m_cn5, m_cn6, m_cn7, m_cn8, m_cn9;

    double c_beta; // drift angle
    double c_rpp; // yaw adim velocioty

    double c_Xpp, c_Xpp0, c_Xpp1;
    double c_Ypp, c_Ypp0, c_Ypp1, c_Ypp2, c_Ypp3, c_Ypp4, c_Ypp5, c_Ypp6, c_Ypp7, c_Ypp8;
    double c_Npp, c_Npp0, c_Npp1, c_Npp2, c_Npp3, c_Npp4, c_Npp5, c_Npp6, c_Npp7, c_Npp8, c_Npp9;

    double m_shipLength;
    double m_shipDraft;
    double m_Cm;
    double m_mu22;

  };

  std::shared_ptr<FrSutuloManoeuvringForce>
  make_Sutulo_manoeuvring_model(const std::string &name, FrBody *body, const std::string &file);

}// end namespace frydom

#endif //FRYDOM_FRSUTULOMANOEUVRINGFORCE_H
