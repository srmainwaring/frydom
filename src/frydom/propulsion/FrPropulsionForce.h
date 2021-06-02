//
// Created by lletourn on 21/05/2021.
//

#ifndef FRYDOM_FRPROPULSIONFORCE_H
#define FRYDOM_FRPROPULSIONFORCE_H

#include "frydom/core/force/FrForce.h"

namespace frydom {

  class FrPropulsionForce : public FrForce {

   public:

    FrPropulsionForce(const std::string& name, FrBody *body, double thrustDeductionFactor, double LOA, double ku);

    void SetStraightRunWakeFraction(double wp0);

    double GetStraightRunWakeFraction() const;

    void SetK1(double k1);

    double GetK1() const;

    void SetThrustDeductionFactor(double tp);

    double GetThrustDeductionFactor() const;

    void SetCorrectionFactor(double ku);

    double GetCorrectionFactor() const;

    double GetLongitudinalVelocity() const;

    double GetTransversalVelocity() const;

   protected:

    void ComputeLongitudinalVelocity();

    double GetSideWashAngle() const;

    double GetWakeFraction() const;

    Position m_positionInBody;

    double m_shipLOA;
    double m_K1;
    double m_correction_factor;
    double m_wake_fraction0;
    double m_thrust_deduction_factor;
    double m_longitudinal_velocity;
    double m_transversal_velocity;

  };

} // end namespace frydom
#endif //FRYDOM_FRPROPULSIONFORCE_H
