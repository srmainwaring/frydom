//
// Created by lletourn on 25/05/2021.
//

#ifndef FRYDOM_FRRUDDERFORCE_H
#define FRYDOM_FRRUDDERFORCE_H

#include <frydom/core/common/FrNode.h>
#include "FrActuatorForceBase.h"
#include "MathUtils/Interp1d.h"
#include "frydom/propulsion/FrPropellerRudder.h"
#include "frydom/core/math/functions/FrFunctionsInc.h"

namespace frydom {

  // Forward declaration
  class FrPropellerForce;


  class FrRudderForce : public FrActuatorForceBase {

   public:

    FrRudderForce(const std::string &name, FrBody *body, const std::shared_ptr<FrNode> &node,
                  const std::string &fileCoefficients);

    void SetProjectedLateralArea(double area);

    double GetProjectedLateralArea() const;

    void SetHeight(double h);

    double GetHeight() const;

    void SetRootChord(double br);

    double GetRootChord() const;

    void SetRampSlope(double value, FREQUENCY_UNIT unit);

    double GetRampSlope(FREQUENCY_UNIT unit) const;

    virtual double GetLiftCoefficient(double attackAngle) const;

    virtual double GetDragCoefficient(double attackAngle) const;

    virtual double GetTorqueCoefficient(double attackAngle) const;

    void SetRudderAngle(double angle, ANGLE_UNIT unit);

    double GetRudderAngle(ANGLE_UNIT unit) const;

    void SetStraightRunWakeFraction(double w0);

    double GetStraightRunWakeFraction() const;

    void SetTransverseVelocityCorrection(double k);

    Position GetPositionInBody() const;

    Velocity GetRudderRelativeVelocityInWorld() const;

    FrFrame GetRudderFlowFrame(const Velocity &rudderFlowVelocity) const;

    double GetAttackAngle(const Velocity &rudderVelocity) const;

    double GetDriftAngle(const Velocity &rudderVelocity) const;

    double GetWakeFraction(double sidewashAngle) const;

    double Kappa(double specialSidewashAngle) const;

    double ComputeSpecialSidewashAngle() const;

    void ActivateHullRudderInteraction(bool interaction) { has_hullRudderInteraction = interaction; }

    void ActivateHullInfluenceOnTransverseVelocity(bool val) {has_hull_influence_transverse_velocity = val;}

   protected:

    double GetDrag() const;
    double GetLift() const;
    double GetTorque() const;

    void DefineLogMessages() override;

    void Compute(double time) override;

    void Initialize() override;

    virtual void ReadCoefficientsFile();

    virtual GeneralizedForce ComputeGeneralizedForceInWorld(const Velocity &rudderRelativeVelocity, double kd=1) const;

    std::string m_reference;

    double m_wakeFraction0;
    double m_K1;

    double m_k;
    double m_beta1;
    double m_beta2;
    double m_K2;
    double m_K3;

    double m_ramp_slope;
    FrFunctionBase *m_rudderAngle;

    double m_projectedLateralArea;
    double m_height;
    double m_rootChord;

    mathutils::LookupTable1D<double, double> m_coefficients;

    std::shared_ptr<FrNode> m_rudderNode;

    bool has_hullRudderInteraction;
    bool has_hull_influence_transverse_velocity;

    std::string c_fileCoefficients;

    mutable double c_drag, c_lift, c_torque;

    friend class FrPropellerRudder;

  };

  std::shared_ptr<FrRudderForce>
  make_rudder_force(const std::string &name, const std::shared_ptr<FrBody> &body, const std::shared_ptr<FrNode> &node,
                    const std::string &fileCoefficients);

} // end namespace frydom
#endif //FRYDOM_FRRUDDERFORCE_H
