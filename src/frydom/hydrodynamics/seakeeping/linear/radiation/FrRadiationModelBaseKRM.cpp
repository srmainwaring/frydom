// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================
#include <memory>

#include "FrRadiationModelBaseKRM.h"
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrRadiationModel.h"

namespace frydom {
  namespace internal {

    FrRadiationModelBaseKRM::~FrRadiationModelBaseKRM() {
    }

    FrRadiationModelBaseKRM::FrRadiationModelBaseKRM(FrRadiationModel *radiationModel) :
      m_frydomRadiationModel(radiationModel) {
      // Instanciate added mass
      m_added_mass = std::make_shared<FrAddedMassBase>(radiationModel);
      // Adding element (added mass) to mesh
      this->AddElement(m_added_mass);
    }
    void FrRadiationModelBaseKRM::SetupInitial() {
      m_added_mass->Initialize();
      this->Setup();
    }

    std::shared_ptr<FrRadiationModelBaseKRM> GetChronoAddedMass(std::shared_ptr<FrRadiationModel> radiationModel) {
      return radiationModel->m_chronoAddedMassModel;
    }

  } // end namespace internal
} // end namespace frydom

