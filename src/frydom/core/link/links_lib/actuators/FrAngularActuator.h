//
// Created by lletourn on 07/05/19.
//

#ifndef FRYDOM_FRANGACTUATOR_H
#define FRYDOM_FRANGACTUATOR_H


#include "FrActuator.h"

//#include "chrono/physics/ChLinkMotorRotation.h"

namespace chrono {
  class ChLinkMotorRotation;
}

namespace frydom {

  // Forward declaration
  class FrAngularActuator;

  namespace internal {

    std::shared_ptr<chrono::ChLinkMotorRotation> GetChronoActuator(std::shared_ptr<FrAngularActuator> actuator);

    std::shared_ptr<chrono::ChLinkMotorRotation> GetChronoActuator(FrAngularActuator *actuator);

  }  // end namespace frydom::internal

  // Forward declaration
  class FrLink;

  class FrAngularActuator : public FrActuator {
   private:
    std::shared_ptr<chrono::ChLinkMotorRotation> m_chronoActuator;

   public:
    FrAngularActuator(const std::string &name, FrLink *actuatedLink, ACTUATOR_CONTROL control);

    void SetMotorFunction(const FrFunctionBase &function) override;

    Force GetMotorForceInNode(FRAME_CONVENTION fc) const override;

    Torque GetMotorTorqueInNode(FRAME_CONVENTION fc) const override;

    double GetMotorPower() const override;

    /// Tells if all constraints of this link are currently turned on or off by the user.
    bool IsDisabled() const override;

    /// User can use this to enable/disable all the constraint of the link as desired.
    void SetDisabled(bool disabled) override;

    void Initialize() override;

    void StepFinalize() override {};

   protected:

    void DefineLogMessages() override;

    friend std::shared_ptr<chrono::ChLinkMotorRotation> internal::GetChronoActuator(std::shared_ptr<FrAngularActuator>);

    friend std::shared_ptr<chrono::ChLinkMotorRotation> internal::GetChronoActuator(FrAngularActuator *actuator);

  };

} //end namespace frydom

#endif //FRYDOM_FRANGACTUATOR_H
