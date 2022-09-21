//
// Created by lletourn on 03/09/2021.
//

#ifndef FRYDOM_FRPROPELLERRUDDER_H
#define FRYDOM_FRPROPELLERRUDDER_H

#include "FrActuatorForceBase.h"
#include "FrPropellerType.h"
#include "acme/acme.h"
#include "frydom/core/math/functions/FrFunctionsInc.h"

namespace frydom {

  class FrPropellerRudder : public FrActuatorForceBase {

   public:

    FrPropellerRudder(const std::string &name,
                      PropellerRudderModelType proprudder_type,
                      PropellerModelType prop_type,
                      const std::shared_ptr<FrNode> &propeller_node,
                      PropellerParams prop_params,
                      RudderModelType rudder_type,
                      const std::shared_ptr<FrNode> &rudder_node,
                      RudderParams rudder_params);

    void SetRPM(double rpm);

    // For CPP only
    void SetPitchRatio(double pitch_ratio);

    double GetRudderAngle(ANGLE_UNIT unit) const;

    // TODO : move to VSL
    void SetRudderRampSlope(double slope) { m_ramp_slope = slope; }

    void SetRudderCommandAngle(double angle, ANGLE_UNIT unit);

    // Propeller Force

    Force GetPropellerForceInWorld(FRAME_CONVENTION fc) const;

    Force GetPropellerForceInBody(FRAME_CONVENTION fc) const;

    Torque GetPropellerTorqueInWorldAtPropeller(FRAME_CONVENTION fc) const;

    Torque GetPropellerTorqueInBodyAtPropeller(FRAME_CONVENTION fc) const;

    Torque GetPropellerTorqueInWorldAtCOG(FRAME_CONVENTION fc) const;

    Torque GetPropellerTorqueInBodyAtCOG(FRAME_CONVENTION fc) const;

    double GetPropulsivePower() const;

    // Rudder Force

    Force GetRudderForceInWorld(FRAME_CONVENTION fc) const;

    Force GetRudderForceInBody(FRAME_CONVENTION fc) const;

    Torque GetRudderTorqueInWorldAtRudder(FRAME_CONVENTION fc) const;

    Torque GetRudderTorqueInBodyAtRudder(FRAME_CONVENTION fc) const;

    Torque GetRudderTorqueInWorldAtCOG(FRAME_CONVENTION fc) const;

    Torque GetRudderTorqueInBodyAtCOG(FRAME_CONVENTION fc) const;

   private:

    void SetRudderAngle(double rudder_angle, ANGLE_UNIT unit);

    void Initialize() override;

    void Compute(double time) override;

    void DefineLogMessages() override;

    std::shared_ptr<acme::PropellerRudderBase> m_acme_propeller_rudder;
    std::shared_ptr<FrNode> m_propeller_node;
    std::shared_ptr<FrNode> m_rudder_node;

    double m_rudder_angle_deg;
    double m_rpm, m_pitch_ratio;

    // TODO : move to VSL
    double m_ramp_slope;
    FrFunctionBase *m_rudderAngleFunction;

    // cached
    double c_water_density;
    double c_x_pr; ///< distance from propeller to rudder (positive if rudder behind the propeller)
    double c_x_gr; ///< distance from COG to rudder (positive if rudder is behind COG)

//    double c_uR0, c_vR0;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  std::shared_ptr<FrPropellerRudder>
  make_propeller_rudder_model(const std::string &name,
                              PropellerRudderModelType proprudder_type,
                              PropellerModelType prop_type,
                              const std::shared_ptr<FrNode> &propeller_node,
                              PropellerParams prop_params,
                              RudderModelType rudder_type,
                              const std::shared_ptr<FrNode> &rudder_node,
                              RudderParams rudder_params,
                              const std::string &prop_input_filepath = "",
                              const std::string &rudder_input_filepath = "");

}
#endif //FRYDOM_FRPROPELLERRUDDER_H
