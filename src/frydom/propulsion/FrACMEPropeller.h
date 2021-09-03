//
// Created by lletourn on 20/08/2021.
//

#ifndef FRYDOM_FRACMEPROPELLER_H
#define FRYDOM_FRACMEPROPELLER_H

#include "FrActuatorForceBase.h"
#include "acme/propeller/PropellerBaseModel.h"
#include "FrPropellerType.h"

namespace frydom {

  class FrACMEPropeller : public FrActuatorForceBase {

   public:

    FrACMEPropeller(const std::string &name, const std::shared_ptr<FrNode> &propeller_node, PropellerParams params,
                    const std::string &perf_data_json_string, PropellerType type);

    void SetRPM(double rpm);

    // For CPP only
    void SetPitchRatio(double pitch_ratio);

    double GetThrust() const {return m_acme_propeller->GetThrust();}

    double GetTorque() const {return m_acme_propeller->GetTorque();}

    double GetPower() const {return m_acme_propeller->GetPower();}

    double GetEfficiency() const {return m_acme_propeller->GetPropellerEfficiency();}

   private:

    void Initialize() override;

    void Compute(double time) override;

    void DefineLogMessages() override;

    std::shared_ptr<FrNode> m_node;

    double m_rpm;
    double m_pitch_ratio;

    std::unique_ptr<acme::PropellerBaseModel> m_acme_propeller;

    // cached
    double c_advance_ratio;
    double c_water_density;
    double c_uP0, c_vP0;

  };

  std::shared_ptr<FrACMEPropeller>
  make_ACME_propeller(const std::string &name, const std::shared_ptr<FrNode> &propeller_node, PropellerParams params,
                      const std::string &perf_data_json_string, PropellerType type);

  std::shared_ptr<FrACMEPropeller> make_ACME_propeller(const std::string &name, const std::shared_ptr<FrNode> &propeller_node,
                                                       double diameter, double wake_fraction,
                                                       double thrust_deduction_factor,
                                                       SCREW_DIRECTION screwDirection,
                                                       const std::string &perf_data_json_string,
                                                       PropellerType type);


}

#endif //FRYDOM_FRACMEPROPELLER_H
