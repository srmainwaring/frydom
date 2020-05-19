//
// Created by lletourn on 07/05/19.
//

#ifndef FRYDOM_FRLINACTUATOR_H
#define FRYDOM_FRLINACTUATOR_H

#include "FrActuator.h"

#include "chrono/physics/ChLinkMotorLinear.h"

namespace frydom {

  class FrLinearActuator;

  namespace internal {

    std::shared_ptr<chrono::ChLinkMotorLinear> GetChronoActuator(std::shared_ptr<FrLinearActuator> actuator);

    std::shared_ptr<chrono::ChLinkMotorLinear> GetChronoActuator(FrLinearActuator *actuator);

  }  // end namespace frydom::internal

  // Forward declaration
  class FrLink;

  class FrLinearActuator : public FrActuator {
   private:
    std::shared_ptr<chrono::ChLinkMotorLinear> m_chronoActuator;

   public:
    FrLinearActuator(const std::string &name, FrLink *actuatedLink, ACTUATOR_CONTROL control);

    void SetMotorFunction(const FrFunctionBase &function) override;

    Force GetMotorForceInNode(FRAME_CONVENTION fc) const override;

    Torque GetMotorTorqueInNode(FRAME_CONVENTION fc) const override;

    double GetMotorPower() const override;

    void Initialize() override;

    /// Tells if all constraints of this link are currently turned on or off by the user.
    bool IsDisabled() const override;

    /// User can use this to enable/disable all the constraint of the link as desired.
    void SetDisabled(bool disabled) override;

   protected:

    void DefineLogMessages() override;

    friend std::shared_ptr<chrono::ChLinkMotorLinear> internal::GetChronoActuator(std::shared_ptr<FrLinearActuator>);

    friend std::shared_ptr<chrono::ChLinkMotorLinear> internal::GetChronoActuator(FrLinearActuator *actuator);

  };

} //end namespace frydom


#endif //FRYDOM_FRLINACTUATOR_H
