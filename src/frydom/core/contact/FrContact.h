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

#ifndef FRYDOM_FRCONTACT_H
#define FRYDOM_FRCONTACT_H

namespace chrono {
  class ChMaterialSurface;
}

namespace frydom {

  // Foward declaration
  class FrContactParamsSMC;
  class FrContactParamsNSC;

  class FrContactParams {

    public:

    FrContactParams();

    float static_friction;
    float sliding_friction;
    float restitution;

    virtual void Print() const = 0;

  };


  namespace internal {
    std::shared_ptr<chrono::ChMaterialSurface> GetChronoMaterial(FrContactParams* params);
    std::shared_ptr<chrono::ChMaterialSurface> GetChronoMaterial(FrContactParamsSMC* params);
    std::shared_ptr<chrono::ChMaterialSurface> GetChronoMaterial(FrContactParamsNSC* params);
  }


} // end namespace frydom


#endif //FRYDOM_FRCONTACT_H
