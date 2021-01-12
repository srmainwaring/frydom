//
// Created by camille on 27/03/2020.
//

#include "FrBuoyancyBarElements.h"

namespace frydom {

  FrBuoyancyBarElements::FrBuoyancyBarElements(const std::string &name, frydom::FrBody *body,
                                               std::shared_ptr<FrBarElementBase> elements)
                                               :  FrForce(name, "BuoyancyBarElement", body),
                                               m_elements(elements) { }

  void FrBuoyancyBarElements::Initialize() {
    m_elements->Initialize();
    FrForce::Initialize();
  }

  void FrBuoyancyBarElements::Compute(double time) {
    m_elements->Update(time);
    SetForceInWorldAtCOG(m_elements->GetForce(), NWU);
    SetTorqueInWorldAtCOG(m_elements->GetTorque(), NWU);
  }

  std::shared_ptr<FrBuoyancyBarElements>
  make_buoyancy_bar_elements(const std::string& name, std::shared_ptr<FrBody> body,
                             std::shared_ptr<FrBarElementBase> elements) {
    assert(body.get() == elements->GetNode()->GetBody());
    auto buoyancy_force = std::make_shared<FrBuoyancyBarElements>(name, body.get(), elements);
    body->AddExternalForce(buoyancy_force);
    return buoyancy_force;
  }

} // end namespace frydom