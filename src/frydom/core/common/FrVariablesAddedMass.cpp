//
// Created by camille on 10/04/2020.
//

#include "frydom/core/common/FrVariablesAddedMass.h"

namespace frydom {

  namespace internal {

    FrVariablesAddedMass::FrVariablesAddedMass() : FrVariablesBodyBase() {}

    FrVariablesAddedMass::FrVariablesAddedMass(const mathutils::Matrix66<double>& added_mass,
                                               chrono::ChVariablesBodyOwnMass* variables)
                                               : FrVariablesBodyBase(variables) {
      m_body_mass = m_mass;
      SetAddedMass(added_mass);
    }

    void FrVariablesAddedMass::SetAddedMass(const mathutils::Matrix66<double>& added_mass) {
      m_added_mass = added_mass;
      auto genMass = m_body_mass + m_added_mass;
      m_mass = genMass;
    }

    mathutils::Matrix66<double>& FrVariablesAddedMass::GetAddedMass() {
      return m_added_mass;
    }

    const mathutils::Matrix66<double>& FrVariablesAddedMass::GetAddedMass() const {
      return m_added_mass;
    }

  } // end namespace internal
} // end namespace frydom
