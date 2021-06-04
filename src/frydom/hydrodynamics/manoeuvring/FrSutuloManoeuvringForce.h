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

    FrSutuloManoeuvringForce(const std::string &name, FrBody* body, const std::string &file);

    void Initialize() override;

    void SetAddedMassCoefficient(double A22) {m_A22 = A22;}
    double GetAddedMassCoefficient() const {return m_A22;}

    void SetShipCharacteristics(double length, double draft);
    double GetShipLength() const;
    double GetShipDraft() const;

    void SetCorrectionFactor(double Cm);
    double GetCorrectionFactor() const;

    void SetManoeuvringCoefficients(std::vector<double> cy, std::vector<double> cn);

   private:

    double ComputeShipDriftAngle();

    double ComputeYawAdimVelocity();

    void LoadResistanceCurve(const std::string& filepath);

    void LoadManoeuvringData(const std::string& filepath);

    void Compute(double time) override;

    double Rh(double u) const;

    std::string c_filepath; ///< path to the JSON file containing the manoeuvring data

    mathutils::Interp1dLinear<double, double> m_hullResistance;
    std::vector<double> m_cy;
    std::vector<double> m_cn;

    double m_shipLength;
    double m_shipDraft;
    double m_Cm;
    double m_A22;

  };

}// end namespace frydom

#endif //FRYDOM_FRSUTULOMANOEUVRINGFORCE_H
