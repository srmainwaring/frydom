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

#include "FrMorisonModelBaseKRM.h"
#include "frydom/hydrodynamics/morison/FrMorisonElements.h"
#include "frydom/hydrodynamics/morison/FrMorisonAddedMass.h"

namespace frydom {
  namespace internal {

    FrMorisonModelBaseKRM::FrMorisonModelBaseKRM(FrMorisonCompositeElement* morisonModel) :
      m_frydomMorisonModel(morisonModel) {
        m_added_mass = std::make_shared<FrMorisonAddedMassBase>(morisonModel);
        this->AddElement(m_added_mass);
    }

    void FrMorisonModelBaseKRM::SetupInitial() {
      m_added_mass->Initialize();
      this->Setup();
    }

    void FrMorisonModelBaseKRM::Update() {
      m_added_mass->Update();
    }

    std::shared_ptr<FrMorisonModelBaseKRM> GetChronoMorisonAddedMass(std::shared_ptr<FrMorisonCompositeElement> morisonModel) {
      return morisonModel->m_chronoAddedMass;
    }

  } // end namespace internal
} // end namespace frydom

