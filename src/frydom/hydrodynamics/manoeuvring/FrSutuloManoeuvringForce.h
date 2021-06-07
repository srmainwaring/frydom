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

//    void SetManoeuvringCoefficients(std::vector<double> cy, std::vector<double> cn);

    double cy0() const {return m_cy0;}
    void cy0(double cy) {m_cy0=cy;}
    double cy1() const {return m_cy1;}
    void cy1(double cy) {m_cy1=cy;}
    double cy2() const {return m_cy2;}
    void cy2(double cy) {m_cy2=cy;}
    double cy3() const {return m_cy3;}
    void cy3(double cy) {m_cy3=cy;}
    double cy4() const {return m_cy4;}
    void cy4(double cy) {m_cy4=cy;}
    double cy5() const {return m_cy5;}
    void cy5(double cy) {m_cy5=cy;}
    double cy6() const {return m_cy6;}
    void cy6(double cy) {m_cy6=cy;}
    double cy7() const {return m_cy7;}
    void cy7(double cy) {m_cy7=cy;}
    double cy8() const {return m_cy8;}
    void cy8(double cy) {m_cy8=cy;}

    double cn0() const {return m_cn0;}
    void cn0(double cn) {m_cn0=cn;}
    double cn1() const {return m_cn1;}
    void cn1(double cn) {m_cn1=cn;}
    double cn2() const {return m_cn2;}
    void cn2(double cn) {m_cn2=cn;}
    double cn3() const {return m_cn3;}
    void cn3(double cn) {m_cn3=cn;}
    double cn4() const {return m_cn4;}
    void cn4(double cn) {m_cn4=cn;}
    double cn5() const {return m_cn5;}
    void cn5(double cn) {m_cn5=cn;}
    double cn6() const {return m_cn6;}
    void cn6(double cn) {m_cn6=cn;}
    double cn7() const {return m_cn7;}
    void cn7(double cn) {m_cn7=cn;}
    double cn8() const {return m_cn8;}
    void cn8(double cn) {m_cn8=cn;}

   private:

    double ComputeShipDriftAngle();

    double ComputeYawAdimVelocity();

    void LoadResistanceCurve(const std::string& filepath);

    void LoadManoeuvringData(const std::string& filepath);

    void Compute(double time) override;

    double Rh(double u) const;

    std::string c_filepath; ///< path to the JSON file containing the manoeuvring data

    mathutils::Interp1dLinear<double, double> m_hullResistance;
    double m_cy0, m_cy1, m_cy2, m_cy3, m_cy4, m_cy5, m_cy6, m_cy7, m_cy8;
    double m_cn0, m_cn1, m_cn2, m_cn3, m_cn4, m_cn5, m_cn6, m_cn7, m_cn8, m_cn9;

//    std::vector<double> m_cy;
//    std::vector<double> m_cn;

    double m_shipLength;
    double m_shipDraft;
    double m_Cm;
    double m_A22;

  };

}// end namespace frydom

#endif //FRYDOM_FRSUTULOMANOEUVRINGFORCE_H
