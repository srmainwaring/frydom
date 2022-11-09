//
// Created by lletourn on 07/05/19.
//

#include "FrLinearActuator.h"

#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"

#include "frydom/core/math/functions/FrFunctionsInc.h"
#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/logging/FrLogManager.h"
#include "frydom/logging/FrTypeNames.h"

namespace frydom {

  FrLinearActuator::~FrLinearActuator() {
  }

  FrLinearActuator::FrLinearActuator(const std::string &name,
                                     FrLink *actuatedLink,
                                     ACTUATOR_CONTROL control)
      : FrActuator(name, TypeToString(this), actuatedLink) {

    switch (control) {
      case POSITION :
        m_chronoActuator = std::make_shared<chrono::ChLinkMotorLinearPosition>();
        break;
      case VELOCITY :
        m_chronoActuator = std::make_shared<chrono::ChLinkMotorLinearSpeed>();
        break;
      case FORCE :
        m_chronoActuator = std::make_shared<chrono::ChLinkMotorLinearForce>();
        break;
    }
    m_chronoActuator->SetGuideConstraint(chrono::ChLinkMotorLinear::GuideConstraint::FREE);

  }

  void FrLinearActuator::SetMotorFunction(const FrFunctionBase &function) {

    auto chronoFunctionInterface = internal::FrFunctionChronoInterface(function);

    m_chronoActuator->SetMotorFunction(chronoFunctionInterface.GetChronoFunction());

  }

  void FrLinearActuator::Initialize() {

    // IMPORTANT : in FRyDoM the first node is the master and the second one the slave, as opposed to Chrono !!!
    // ChLinkMotorLinear motorized along the x axis, while ChLinkLock::Prismatic is along z axis...
    auto frame1 = GetNode1()->GetFrameWRT_COG_InBody();
    frame1.RotY_RADIANS(MU_PI_2, NWU, true);
    auto frame2 = GetNode2()->GetFrameWRT_COG_InBody();
    frame2.RotY_RADIANS(MU_PI_2, NWU, true);

    m_chronoActuator->Initialize(internal::GetChronoBody2(this),
                                 internal::GetChronoBody1(this),
                                 true,
                                 internal::FrFrame2ChFrame(frame2), internal::FrFrame2ChFrame(frame1));

//      GetSystem()->GetLogManager()->Add(this);
  }

  bool FrLinearActuator::IsDisabled() const {
    return m_chronoActuator->IsDisabled(); // TODO : voir si on teste aussi m_actuatedLink
  }

  void FrLinearActuator::SetDisabled(bool disabled) {
    m_chronoActuator->SetDisabled(disabled);
  }

  double FrLinearActuator::GetMotorPower() const {
    return m_chronoActuator->GetMotorForce() * m_chronoActuator->GetMotorPos_dt();
  }

  Force FrLinearActuator::GetMotorForceInNode(FRAME_CONVENTION fc) const {
    return {0., 0., m_chronoActuator->GetMotorForce()};
  }

  Torque FrLinearActuator::GetMotorTorqueInNode(FRAME_CONVENTION fc) const {
    return {};
  }

  void FrLinearActuator::DefineLogMessages() {

    auto msg = NewMessage("State", "State message");

    msg->AddField<double>
        ("MotorPower", "kW", "power delivered by the motor", [this]() { return GetMotorPower(); });
    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("MotorForceInBody1", "N",
         fmt::format("Force applied by the motor on body 1, in body 1 reference frame {}", GetLogFC()),
         [this]() { return GetMotorForceInBody1(GetLogFC()); });
    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("MotorForceInBody2", "N",
         fmt::format("Force applied by the motor on body 1, in body 2 reference frame {}", GetLogFC()),
         [this]() { return GetMotorForceInBody2(GetLogFC()); });

  }

  namespace internal {

    std::shared_ptr<chrono::ChLinkMotorLinear> GetChronoActuator(std::shared_ptr<FrLinearActuator> actuator) {
      return actuator->m_chronoActuator;
    }

    std::shared_ptr<chrono::ChLinkMotorLinear> GetChronoActuator(FrLinearActuator *actuator) {
      return actuator->m_chronoActuator;
    }

  }  // end namespace frydom::internal

} // end namespace frydom
