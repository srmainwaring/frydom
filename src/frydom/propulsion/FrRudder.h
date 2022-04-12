//
// Created by lletourn on 20/08/2021.
//

#ifndef FRYDOM_FRRUDDER_H
#define FRYDOM_FRRUDDER_H

#include "FrActuatorForceBase.h"
#include "FrPropellerType.h"
#include "acme/rudder/SimpleRudderModel.h"
#include "frydom/core/math/functions/FrFunctionsInc.h"

namespace frydom {

  class FrRudder : public FrActuatorForceBase {

   public:

    FrRudder(const std::string &name,
             const std::shared_ptr<FrNode> &rudder_node,
             RudderParams params,
             const std::string &perf_data_json_string,
             RudderModelType type);

    void SetRudderAngle(double rudder_angle, ANGLE_UNIT unit);

    double GetRudderAngle(ANGLE_UNIT unit) const;

    // TODO : move to VSL
    void SetRudderRampSlope(double slope) { m_ramp_slope = slope; }

    void SetRudderCommandAngle(double rudder_angle, ANGLE_UNIT unit);

   private:

    void Initialize() override;

    void Compute(double time) override;

    void StepFinalize() override;

    void DefineLogMessages() override;

    std::shared_ptr<FrNode> m_node;

    std::unique_ptr<acme::RudderBaseModel> m_acme_rudder;

    double m_rudder_angle_deg;

    // cached
    double c_water_density;
    double c_uR0, c_vR0;
    double c_x_gr;

    // TODO : move to VSL
    double m_ramp_slope;
    FrFunctionBase *m_rudderAngleFunction;

  };

  std::shared_ptr<FrRudder>
  make_rudder_model(const std::string &name,
                    const std::shared_ptr<FrNode> &rudder_node,
                    RudderParams params,
                    const std::string &rudder_input_filepath,
                    RudderModelType type);

}
#endif //FRYDOM_FRRUDDER_H
