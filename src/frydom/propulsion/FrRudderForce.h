//
// Created by lletourn on 25/05/2021.
//

#ifndef FRYDOM_FRRUDDERFORCE_H
#define FRYDOM_FRRUDDERFORCE_H

#include <frydom/core/common/FrNode.h>
#include "frydom/core/force/FrForce.h"
#include "MathUtils/Interp1d.h"

namespace frydom {

  class FrPropellerForce;

//  class FrFoilForce : public FrForce {
//
//   public:
//    FrFoilForce(const std::string& name, FrBody* body, const std::string& fileCoefficients);
//
//    void SetProjectedLateralArea(double area);
//
//    double GetProjectedLateralArea() const;
//
//   protected:
//
//    virtual Velocity GetInflowVelocityInWorld() const = 0;
//
//    virtual FrFrame GetInflowFrame(Velocity inflowVelocity) const = 0;
//
//    virtual double GetAttackAngle(Velocity inflowVelocity) const = 0;
//
//    virtual Position GetPositionInBody() const = 0;
//
//    virtual void ReadCoefficientsFile(const std::string& filename) = 0;
//
//    mathutils::Vector3d<double> GetCoefficients(double attackAngle) const;
//
//    virtual GeneralizedForce ComputeGeneralizedForceInWorld(Velocity inflowVelocity) const = 0;
//
//    void Compute(double time) override;
//
//    /// This subroutine initializes the object FrForce.
//    void Initialize() override;
//
//    mathutils::LookupTable1D<double, mathutils::Vector3d<double>> m_coefficients;
//    double m_projectedLateralArea;
//    double c_fluidDensity;
//
//  };

//  class FrRudderForce : public FrFoilForce {
  class FrRudderForce : public FrForce {

   public:

    FrRudderForce(const std::string& name, FrBody* body, const std::string& fileCoefficients,
                  const std::shared_ptr<FrNode>& node);

    void SetProjectedLateralArea(double area);

    double GetProjectedLateralArea() const;

    virtual mathutils::Vector3d<double> GetCoefficients(double attackAngle) const;

    void SetRudderAngle(double angle);

    double GetRudderAngle() const;

    void SetStraightRunWakeFraction(double w0);

    double GetStraightRunWakeFraction() const;

    void SetTransverseVelocityCorrection(double k);

    Position GetPositionInBody() const;

    Velocity GetInflowVelocityInWorld() const;

    FrFrame GetInflowFrame(Velocity inflowVelocity) const;

    double GetAttackAngle(Velocity inflowVelocity) const;

    double GetDriftAngle(Velocity inflowVelocity) const;

    double GetWakeFraction(double sidewashAngle) const;

    double Kappa(double specialSidewashAngle) const;

    double ComputeSpecialSidewashAngle() const;

    void ActivateHullRudderInteraction() { is_hullRudderInteraction = true;}

    void DeactivateHullRudderInteraction() { is_hullRudderInteraction = false;}

   protected:

    void Compute(double time) override;

    /// This subroutine initializes the object FrForce.
    void Initialize() override;

    virtual void ReadCoefficientsFile() = 0;

    virtual GeneralizedForce ComputeGeneralizedForceInWorld(Velocity inflowVelocity) const;

    double m_wakeFraction0;
    double m_K1;

    double m_k;
    double m_beta1;
    double m_beta2;
    double m_K2;
    double m_K3;

    double m_rudderAngle;
    double m_projectedLateralArea;

    mathutils::LookupTable1D<double, mathutils::Vector3d<double>> m_coefficients;

    std::shared_ptr<FrNode> m_rudderNode;

    bool is_hullRudderInteraction;

    std::string c_fileCoefficients;

  };

} // end namespace frydom
#endif //FRYDOM_FRRUDDERFORCE_H
