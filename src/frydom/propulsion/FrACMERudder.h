//
// Created by lletourn on 20/08/2021.
//

#ifndef FRYDOM_FRACMERUDDER_H
#define FRYDOM_FRACMERUDDER_H

#include "FrActuatorForceBase.h"
#include "FrPropellerType.h"
#include "acme/rudder/SimpleRudderModel.h"
#include "frydom/core/math/functions/FrFunctionsInc.h"

namespace frydom {

  class FrACMERudder : public FrActuatorForceBase {

   public:

    FrACMERudder(const std::string &name, const std::shared_ptr<FrNode> &rudder_node, RudderParams params,
                 const std::string &perf_data_json_string, RudderType type);

    void SetRudderAngle(double rudder_angle, ANGLE_UNIT unit);

    double GetRudderAngle(ANGLE_UNIT unit) const;

    // TODO : move to VSL
    void SetRudderRampSlope(double slope) {m_ramp_slope = slope;}
    void SetRudderCommandAngle(double rudder_angle, ANGLE_UNIT unit);

   private:

    void Initialize() override;

    void Compute(double time) override;

    void StepFinalize() override;

    void DefineLogMessages() override;

    std::shared_ptr<FrNode> m_node;

    std::unique_ptr<acme::SimpleRudderModel> m_acme_rudder;

    double m_rudder_angle_deg;

    // cached
    double c_water_density;

    double c_uR0, c_vR0;

    // TODO : move to VSL
    double m_ramp_slope;
    FrFunctionBase *m_rudderAngleFunction;

  };

  std::shared_ptr<FrACMERudder>
  make_ACME_rudder(const std::string &name, const std::shared_ptr<FrNode> &rudder_node, RudderParams params,
                   const std::string &perf_data_json_string, RudderType type);

  std::shared_ptr<FrACMERudder>
  make_ACME_rudder(const std::string &name, const std::shared_ptr<FrNode> &rudder_node, double area_m2, double chord_m, double height_m,
                   double wake_fraction, const std::string &perf_data_json_string, RudderType type);

}
#endif //FRYDOM_FRACMERUDDER_H
