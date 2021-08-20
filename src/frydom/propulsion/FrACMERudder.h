//
// Created by lletourn on 20/08/2021.
//

#ifndef FRYDOM_FRACMERUDDER_H
#define FRYDOM_FRACMERUDDER_H

#include "FrActuatorForceBase.h"
#include "acme/rudder/SimpleRudderModel.h"

namespace frydom {

  using RudderType = acme::RudderModelType;
  using RudderParams = acme::RudderParams;

  class FrACMERudder : public FrActuatorForceBase {

   public:

    FrACMERudder(const std::string &name, const FrNode &rudder_node, RudderParams params,
                 const std::string &perf_data_json_string, RudderType type);

    void SetRudderAngle(double rudder_angle);

   private:

    void Initialize() override;

    void Compute(double time) override;

    void DefineLogMessages() override;

    std::shared_ptr<FrNode> m_node;

    std::unique_ptr<acme::SimpleRudderModel> m_acme_rudder;

    double m_rudder_angle;

    // cached
    double c_water_density;

    double c_uR0, c_vR0;

  };

  std::shared_ptr<FrACMERudder>
  make_ACME_rudder(const std::string &name, const FrNode &rudder_node, RudderParams params,
                   const std::string &perf_data_json_string, RudderType type);

  std::shared_ptr<FrACMERudder>
  make_ACME_rudder(const std::string &name, const FrNode &rudder_node, double area_m2, double chord_m, double height_m,
                   double wake_fraction, const std::string &perf_data_json_string, RudderType type);

}
#endif //FRYDOM_FRACMERUDDER_H
