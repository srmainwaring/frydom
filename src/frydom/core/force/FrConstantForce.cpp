//
// Created by frongere on 07/05/2020.
//


#include "FrConstantForce.h"

#include "frydom/logging/FrTypeNames.h"
#include "frydom/core/common/FrNode.h"


namespace frydom {


  FrConstantForce::FrConstantForce(const std::string &name,
                                   std::shared_ptr<FrNode> node,
                                   MODE mode,
                                   const Force &force,
                                   FRAME_CONVENTION fc) :
      FrForce(name, TypeToString(this), node->GetBody()),
      m_node(node),
      m_mode(mode),
      m_force((IsNED(fc) ? internal::SwapFrameConvention(force) : force)) {}

  void FrConstantForce::SetForce(const Force &force, FRAME_CONVENTION fc) {
    m_force = IsNED(fc) ? internal::SwapFrameConvention(force) : force;
  }

  Force FrConstantForce::GetForce() const {
    return m_force;
  }

  FrConstantForce::MODE FrConstantForce::GetMode() const {
    return m_mode;
  }

  void FrConstantForce::SetMode(MODE mode) {
    m_mode = mode;
  }

  void FrConstantForce::SwitchMode() {
    switch (m_mode) {
      case ABSOLUTE:
        m_mode = FOLLOWING;
        break;
      case FOLLOWING:
        m_mode = ABSOLUTE;
        break;
    }
  }

  void FrConstantForce::Compute(double time) {
    Position node_position = m_node->GetPositionInWorld(NWU);
    switch (m_mode) {
      case ABSOLUTE:
        SetForceInWorldAtPointInWorld(m_force, node_position, NWU);
        break;
      case FOLLOWING:
        SetForceInBodyAtPointInWorld(m_force, node_position, NWU);
        break;
    }
  }


  std::shared_ptr<FrConstantForce> make_constant_force(const std::string &name,
                                                       std::shared_ptr<FrNode> node,
                                                       FrConstantForce::MODE mode,
                                                       const Force &force,
                                                       FRAME_CONVENTION fc) {
    auto constant_force = std::make_shared<FrConstantForce>(name,
                                                            node,
                                                            mode,
                                                            force,
                                                            fc);
    node->GetBody()->AddExternalForce(constant_force);

    return constant_force;
  }


}  // end namespace frydom
