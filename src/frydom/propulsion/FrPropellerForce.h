//
// Created by lletourn on 21/05/2021.
//

#ifndef FRYDOM_FRPROPELLERFORCE_H
#define FRYDOM_FRPROPELLERFORCE_H

#include "frydom/core/force/FrForce.h"
#include "frydom/propulsion/FrPropellerRudder.h"
#include "FrPropellerType.h"
#include "FrRudderForce.h"


namespace frydom {

  class FrPropellerForce : public FrForce {

   public:

    FrPropellerForce(const std::string &name, FrBody *body, Position propellerPositionInBody, FRAME_CONVENTION fc);

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

    Position GetPositionInBody() const;

    void SetRotationalVelocity(double omega, mathutils::FREQUENCY_UNIT unit);

    void SetRPM(double rpm);

    double GetRotationalVelocity(FREQUENCY_UNIT unit) const;

    void SetScrewDirection(SCREW_DIRECTION dir);

    signed int GetScrewDirectionSign() const;

    double GetPower() const;

   protected:

    void Compute(double time) override;

    /// This subroutine initializes the object FrForce.
    void Initialize() override;

    virtual void ReadCoefficientsFile() = 0;

    double ComputeLongitudinalVelocity() const;

    double GetWakeFraction(double sidewashAngle) const;

    virtual GeneralizedForce ComputeGeneralizedForceInWorld() = 0;

    void DefineLogMessages() override;

    std::string m_name;
    std::string m_reference;

    Position m_positionInBody;
    double m_diameter;
    double m_rotational_velocity;
    SCREW_DIRECTION m_screwDirection;

    // Hull/propeller interaction coefficients
    double m_K1;
    double m_correction_factor;
    double m_wake_fraction0;
    double m_thrust_deduction_factor;

    friend class FrPropellerRudder;
  };

} // end namespace frydom
#endif //FRYDOM_FRPROPELLERFORCE_H
