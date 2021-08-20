//
// Created by lletourn on 20/08/2021.
//

#ifndef FRYDOM_FRACMEPROPELLER_H
#define FRYDOM_FRACMEPROPELLER_H

#include "FrActuatorForceBase.h"
#include "acme/propeller/PropellerBaseModel.h"

namespace frydom {

  using PropellerType = acme::PropellerModelType;
  using PropellerParams = acme::PropellerParams;

  class FrACMEPropeller : public FrActuatorForceBase {

   public:

    FrACMEPropeller(const std::string &name, const FrNode &propeller_node, PropellerParams params,
                    const std::string &perf_data_json_string, PropellerType type);

    void SetRPM(double rpm);

    // For CPP only
    void SetPitchRatio(double pitch_ratio);

   private:

    void Compute(double time) override;

    void Initialize() override;

    void DefineLogMessages() override;

    std::shared_ptr<FrNode> m_node;

    double m_rpm;
    double m_pitch_ratio;

    std::unique_ptr<acme::PropellerBaseModel> m_acme_propeller;

    // cached
    double c_water_density;
    double c_thrust;
    double c_torque;
    double c_efficiency;
    double c_power;

    double c_uPA, c_vPA;

  };

  std::shared_ptr<FrACMEPropeller>
  make_ACME_propeller(const std::string &name, const FrNode &propeller_node, PropellerParams params,
                      const std::string &perf_data_json_string, PropellerType type);

  std::shared_ptr<FrACMEPropeller> make_ACME_propeller(const std::string &name, const FrNode &propeller_node,
                                                       double diameter, double wake_fraction,
                                                       double thrust_deduction_factor,
                                                       const std::string &screwDirection,
                                                       const std::string &perf_data_json_string, PropellerType type);


}

#endif //FRYDOM_FRACMEPROPELLER_H
