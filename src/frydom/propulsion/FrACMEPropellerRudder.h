//
// Created by lletourn on 03/09/2021.
//

#ifndef FRYDOM_FRACMEPROPELLERRUDDER_H
#define FRYDOM_FRACMEPROPELLERRUDDER_H

#include "FrActuatorForceBase.h"
#include "FrPropellerType.h"
#include "acme/acme.h"
#include "frydom/core/math/functions/FrFunctionsInc.h"

namespace frydom {

  class FrACMEPropellerRudder : public FrActuatorForceBase {

   public:

    FrACMEPropellerRudder(const std::string& name,
                          PropellerModelType prop_type,
                          const std::shared_ptr<FrNode>& propeller_node,
                          PropellerParams prop_params,
                          const std::string &prop_perf_data_string,
                          RudderModelType rudder_type,
                          const std::shared_ptr<FrNode>& rudder_node,
                          RudderParams rudder_params,
                          const std::string &rudder_perf_data_string);

    void SetRPM(double rpm);

    // For CPP only
    void SetPitchRatio(double pitch_ratio);

    void SetRudderAngle(double rudder_angle, ANGLE_UNIT unit);

    double GetRudderAngle(ANGLE_UNIT unit) const;

    // TODO : move to VSL
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

    void Initialize() override;

    void Compute(double time) override;

    void DefineLogMessages() override;

    std::unique_ptr<acme::PropellerRudderBase> m_acme_propeller_rudder;
    std::shared_ptr<FrNode> m_propeller_node;
    std::shared_ptr<FrNode> m_rudder_node;

    double m_rudder_angle_deg;
    double m_rpm, m_pitch_ratio;

    // TODO : move to VSL
    double m_ramp_slope;
    FrFunctionBase *m_rudderAngleFunction;

    // cached
    double c_water_density;
    double c_xr; ///< distance from propeller to rudder (positive if rudder behind the propeller)

//    double c_uR0, c_vR0;

  };

  std::shared_ptr<FrACMEPropellerRudder> make_ACME_propeller_rudder(const std::string& name,
                                                                    PropellerModelType prop_type,
                                                                    const std::shared_ptr<FrNode>& propeller_node,
                                                                    PropellerParams prop_params,
                                                                    const std::string &prop_input_filepath,
                                                                    RudderModelType rudder_type,
                                                                    const std::shared_ptr<FrNode>& rudder_node,
                                                                    RudderParams rudder_params,
                                                                    const std::string &rudder_input_filepath);

}
#endif //FRYDOM_FRACMEPROPELLERRUDDER_H
