//
// Created by frongere on 12/10/2021.
//

#ifndef FRYDOM_FRABKOWITZMANOEUVRINGFORCE_H
#define FRYDOM_FRABKOWITZMANOEUVRINGFORCE_H

#include "frydom/core/force/FrForce.h"

namespace frydom {

  class FrHullResistance;

  /// Abkowitz manoeuvring model from Yoshimura (2012)
  class FrAbkowitzManoeuvringForce : public FrForce {

   public:
    FrAbkowitzManoeuvringForce(const std::string &name, FrBody *body, const std::string &file,
                               const Position &mid_ship);

    void Initialize() override;

    double GetUMin() const;

    double GetUMax() const;

   private:

    void Compute(double time) override;

    void DefineLogMessages() override;

    void LoadResistanceCurve(const std::string& filepath);

    void LoadManoeuvringData(const std::string& filepath);

   private:

    std::string c_filepath; ///< path to the JSON file containing the manoeuvring data

    std::shared_ptr<FrHullResistance> m_hullResistance;

    double m_Lpp;
    double m_draft;
    Position m_mid_ship;

    double m_Xvv, m_Xvvvv, m_Xrr, m_Xvr;
    double m_Yv, m_Yvvv, m_Yvrr, m_Yr, m_Yrrr, m_Yvvr;
    double m_Nv, m_Nvvv, m_Nvrr, m_Nr, m_Nrrr, m_Nvvr;

  };

  std::shared_ptr<FrAbkowitzManoeuvringForce>
      make_abkowitz_manoeuvring_model(const std::string& name, std::shared_ptr<FrBody> body,
                                      const std::string& file, const Position& mid_ship);


}  // end namespace frydom

#endif //FRYDOM_FRABKOWITZMANOEUVRINGFORCE_H
