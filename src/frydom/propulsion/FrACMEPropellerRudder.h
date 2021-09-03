//
// Created by lletourn on 03/09/2021.
//

#ifndef FRYDOM_FRACMEPROPELLERRUDDER_H
#define FRYDOM_FRACMEPROPELLERRUDDER_H

#include "FrActuatorForceBase.h"
#include "FrPropellerType.h"

#include "acme/acme.h"

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

   private:

    void Initialize() override;

    void Compute(double time) override;

    void DefineLogMessages() override;

    std::unique_ptr<acme::PropellerRudderBase> m_acme_propeller_rudder;
    std::shared_ptr<FrNode> m_propeller_node;
    std::shared_ptr<FrNode> m_rudder_node;

    double m_rudder_angle_deg;
    double m_rpm, m_pitch_ratio;

    // cached
    double c_water_density;
    double c_xr; ///< distance from propeller to rudder

//    double c_uR0, c_vR0;

  };

}
#endif //FRYDOM_FRACMEPROPELLERRUDDER_H
