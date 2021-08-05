//
// Created by frongere on 04/08/2021.
//

#ifndef ACME_THRUSTERBASEMODEL_H
#define ACME_THRUSTERBASEMODEL_H

#include "MathUtils/Vector3d.h"

namespace acme {

  enum SCREW_DIRECTION {
    RIGHT_HANDED,
    LEFT_HANDED
  };

  struct ThrusterBaseParams {
    double m_diameter_m;
    SCREW_DIRECTION m_screw_direction;

    double m_k1;
    double m_correction_factor;
    double m_hull_wake_fraction;
    double m_thrust_deduction_factor;
  };


  class ThrusterBaseModel {

   public:
    explicit ThrusterBaseModel(const ThrusterBaseParams params) :
        m_params(params) {}

//    virtual ~ThrusterBaseModel() = default; // Making it polymorphic

    /// Compute the models with the specified data
    /// \param water_density in kg/m3
    /// \param propeller_advance_velocity_ms propeller velocity with respect to water (current included) along the local
    ///        x-axis of the vessel (in m/s)
    /// \param rpm shaft rotational velocity in round per minutes
    virtual void Compute(const double &water_density,
                         const mathutils::Vector3d<double> &propeller_velocity_through_water,
                         const double &rpm) = 0;

//    virtual double ComputeThrust(const double &water_density,
//                                 const double &advance_velocity_ms,
//                                 const double &rpm) const = 0;
//
//    virtual double ComputeShaftTorque(const double &water_density,
//                                      const double &advance_velocity_ms,
//                                      const double &rpm) const = 0;
//
//    virtual void ComputeThrustAndTorque(const double &water_density,
//                                        const double &advance_velocity_ms,
//                                        const double &rpm,
//                                        double &thrust,
//                                        double &torque) const = 0;


   protected:
    SCREW_DIRECTION GetScrewDirection() const {
      return m_params.m_screw_direction;
    }

    inline signed int GetScrewDirectionSign() const {
      return m_params.m_screw_direction == RIGHT_HANDED ? 1 : -1;
    }


   protected:
    ThrusterBaseParams m_params;

    double c_thrust;
    double c_torque;

  };

}

#endif //ACME_THRUSTERBASEMODEL_H
