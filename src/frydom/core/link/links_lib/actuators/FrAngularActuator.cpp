//
// Created by lletourn on 07/05/19.
//

#include "FrAngularActuator.h"

#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"

#include "frydom/core/math/functions/FrFunctionsInc.h"
#include "frydom/core/common/FrNode.h"

#include "frydom/logging/FrLogManager.h"
#include "frydom/logging/FrTypeNames.h"

#include "frydom/core/body/FrBody.h"

namespace frydom {


  FrAngularActuator::FrAngularActuator(const std::string &name,
                                       FrLink *actuatedLink,
                                       ACTUATOR_CONTROL control)
      : FrActuator(name, TypeToString(this), actuatedLink) {

    switch (control) {
      case POSITION :
        m_chronoActuator = std::make_shared<chrono::ChLinkMotorRotationAngle>();
        break;
      case VELOCITY :
        m_chronoActuator = std::make_shared<chrono::ChLinkMotorRotationSpeed>();
        break;
      case FORCE :
        m_chronoActuator = std::make_shared<chrono::ChLinkMotorRotationTorque>();
        break;
    }
    m_chronoActuator->SetSpindleConstraint(chrono::ChLinkMotorRotation::SpindleConstraint::FREE);

  }

  void FrAngularActuator::SetMotorFunction(const FrFunctionBase &function) {

    auto chronoFunctionInterface = internal::FrFunctionChronoInterface(function);

    m_chronoActuator->SetMotorFunction(chronoFunctionInterface.GetChronoFunction());

  }

  void FrAngularActuator::Initialize() {

    // IMPORTANT : in FRyDoM the first node is the master and the second one the slave, as opposed to Chrono !!!
    m_chronoActuator->Initialize(internal::GetChronoBody2(this),
                                 internal::GetChronoBody1(this),
                                 true,
                                 internal::FrFrame2ChFrame(GetNode2()->GetFrameWRT_COG_InBody()),
                                 internal::FrFrame2ChFrame(GetNode1()->GetFrameWRT_COG_InBody()));

//      GetSystem()->GetLogManager()->Add(this);

  }

  double FrAngularActuator::GetMotorPower() const {
    return m_chronoActuator->GetMotorTorque() * m_chronoActuator->GetMotorRot_dt();
  }

  bool FrAngularActuator::IsDisabled() const {
    return m_chronoActuator->IsDisabled(); // TODO : voir si on teste aussi m_actuatedLink
  }

  void FrAngularActuator::SetDisabled(bool disabled) {
    m_chronoActuator->SetDisabled(disabled);
  }

  Force FrAngularActuator::GetMotorForceInNode(FRAME_CONVENTION fc) const {
    return {};
  }

  Torque FrAngularActuator::GetMotorTorqueInNode(FRAME_CONVENTION fc) const {
    return {0., 0., m_chronoActuator->GetMotorTorque()};
  }

  void FrAngularActuator::DefineLogMessages() {

    auto msg = NewMessage("State", "State message");

    msg->AddField<double>
        ("MotorPower", "kW", "power delivered by the motor", [this]() { return GetMotorPower(); });
    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("MotorTorqueInBody1(", "Nm",
         fmt::format("Torque applied by the motor on body 1, in body 1 reference frame {}", GetLogFC()),
         [this]() { return GetMotorTorqueInBody1(GetLogFC()); });
    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("MotorTorqueInBody2(", "Nm",
         fmt::format("Torque applied by the motor on body 2, in body 2 reference frame {}", GetLogFC()),
         [this]() { return GetMotorTorqueInBody2(GetLogFC()); });
    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("MotorTorqueAtCOGInBody1(", "Nm",
         fmt::format("Torque applied by the motor at COG on body 1, in body 1 reference frame {}", GetLogFC()),
         [this]() { return GetMotorTorqueAtCOGInBody1(GetLogFC()); });
    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("MotorTorqueAtCOGInBody2(", "Nm",
         fmt::format("Torque applied by the motor at COG on body 2, in body 2 reference frame {}", GetLogFC()),
         [this]() { return GetMotorTorqueAtCOGInBody2(GetLogFC()); });

  }


  namespace internal {

    std::shared_ptr<chrono::ChLinkMotorRotation> GetChronoActuator(std::shared_ptr<FrAngularActuator> actuator) {
      return actuator->m_chronoActuator;
    }

    std::shared_ptr<chrono::ChLinkMotorRotation> GetChronoActuator(FrAngularActuator *actuator) {
      return actuator->m_chronoActuator;
    }

  }  // end namespace frydom::internal

} // end namespace frydom
