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

#ifndef FRYDOM_FrRadiationModelBaseKRM_H
#define FRYDOM_FrRadiationModelBaseKRM_H

#include "chrono/fea/ChMesh.h"
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrAddedMass.h"


namespace frydom {

  // Forward declarations
  class FrRadiationModel;

  namespace internal {

    class FrRadiationModelBaseKRM : public chrono::fea::ChMesh {

      public:

      explicit FrRadiationModelBaseKRM(FrRadiationModel* radiationModel);

      void SetupInitial() override;

      FrRadiationModel* GetRadiationModel() const { return m_frydomRadiationModel; }

      std::shared_ptr<FrAddedMassBase> GetAddedMass() { return m_added_mass; }

      private:

        FrRadiationModel* m_frydomRadiationModel;

        std::shared_ptr<FrAddedMassBase> m_added_mass;

    };

    std::shared_ptr<FrRadiationModelBaseKRM> GetChronoAddedMass(std::shared_ptr<FrRadiationModel> radiationModel);

  } // end namespace internal

} // end namespace frydom

#endif //FRYDOM_FrRadiationModelBaseKRM_H