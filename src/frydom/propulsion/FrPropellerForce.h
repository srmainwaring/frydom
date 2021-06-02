//
// Created by lletourn on 21/05/2021.
//

#ifndef FRYDOM_FRPROPELLERFORCE_H
#define FRYDOM_FRPROPELLERFORCE_H

#include "frydom/core/force/FrForce.h"

namespace frydom {

  class FrPropellerForce : public FrForce {

   public:

    FrPropellerForce(const std::string& name, FrBody *body, Position propellerPositionInBody);

    void SetDiameter(double D);;

    double GetDiameter() const;

    void SetStraightRunWakeFraction(double wp0);

    double GetStraightRunWakeFraction() const;

    void SetK1(double k1);

    double GetK1() const;

    void SetThrustDeductionFactor(double tp);

    double GetThrustDeductionFactor() const;

    void SetCorrectionFactor(double ku);

    double GetCorrectionFactor() const;

    double GetLongitudinalVelocity() const;

    Position GetPositionInBody() const;

    virtual GeneralizedForce ComputeGeneralizedForceInWorld() = 0;

    void SetRotationalVelocity(double omega, mathutils::FREQUENCY_UNIT unit);

    double GetRotationalVelocity(FREQUENCY_UNIT unit) const;

   protected:

    void Compute(double time) override;

    /// This subroutine initializes the object FrForce.
    void Initialize() override;

    virtual void ReadCoefficientsFile() = 0;

    double ComputeLongitudinalVelocity();

    double GetWakeFraction(double sidewashAngle) const;

    Position m_positionInBody;
    double m_diameter;
    double m_rotational_velocity;

    // Hull/propeller interaction coefficients
    double m_shipLOA;
    double m_K1;
    double m_correction_factor;
    double m_wake_fraction0;
    double m_thrust_deduction_factor;

    double m_longitudinal_velocity;
//    double m_transversal_velocity;
  };

} // end namespace frydom
#endif //FRYDOM_FRPROPELLERFORCE_H
