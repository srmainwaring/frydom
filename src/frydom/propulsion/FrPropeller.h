//
// Created by lletourn on 20/08/2021.
//

#ifndef FRYDOM_FRPROPELLER_H
#define FRYDOM_FRPROPELLER_H

#include "FrActuatorForceBase.h"
#include "acme/propeller/PropellerBaseModel.h"
#include "FrPropellerType.h"

namespace frydom {

  class FrPropeller : public FrActuatorForceBase {

   public:

    FrPropeller(const std::string &name, const std::shared_ptr<FrNode> &propeller_node, PropellerParams params,
                const std::string &perf_data_json_string, PropellerModelType type);

    void SetRPM(double rpm);

    // For CPP only
    void SetPitchRatio(double pitch_ratio);

    double GetThrust() const { return m_acme_propeller->GetThrust(); }

    double GetTorque() const { return m_acme_propeller->GetTorque(); }

    double GetPower() const { return m_acme_propeller->GetPower(); }

    double GetEfficiency() const { return m_acme_propeller->GetPropellerEfficiency(); }

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

  std::shared_ptr<FrPropeller>
  make_propeller_model(const std::string &name,
                       const std::shared_ptr<FrNode> &propeller_node,
                       PropellerParams params,
                       const std::string &prop_input_filepath,
                       PropellerModelType type);

}

#endif //FRYDOM_FRPROPELLER_H
