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

#ifndef FRYDOM_FRMORISONMODELBASEKRM_H
#define FRYDOM_FRMORISONMODELBASEKRM_H

#include "chrono/fea/ChMesh.h"

namespace frydom {

  // Forward declaration
  class FrMorisonCompositeElement;

  namespace internal {

    // Forward declaration
    class FrMorisonAddedMassBase;

    class FrMorisonModelBaseKRM : public chrono::fea::ChMesh {

      public:

        explicit FrMorisonModelBaseKRM(FrMorisonCompositeElement* morisonModel);

        void SetupInitial() override;

        void Update();

        FrMorisonCompositeElement* GetMorisonModel() { return m_frydomMorisonModel; }

        std::shared_ptr<FrMorisonAddedMassBase> GetAddedMass() { return m_added_mass; }

      private:

        FrMorisonCompositeElement* m_frydomMorisonModel;

        std::shared_ptr<FrMorisonAddedMassBase> m_added_mass;

    };

    std::shared_ptr<FrMorisonModelBaseKRM> GetChronoMorisonAddedMass(std::shared_ptr<FrMorisonCompositeElement> morisonModel);

  } // end namespace internal
} // end namespace frydom

#endif //FRYDOM_FRMORISONMODELBASEKRM_H
