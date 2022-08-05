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

#ifndef FRYDOM_FRMATERIALSURFACENSC_H
#define FRYDOM_FRMATERIALSURFACENSC_H

#include <memory>
#include "FrMaterialSurface.h"

namespace frydom {


  // TODO: utiliser egalement le ChMaterialCompositeNSC pour faire interagir des materiaux differents

  // Forward declaration
  class FrBody;

  class FrMaterialSurfaceNSC: public FrMaterialSurface {

    public:

    FrMaterialSurfaceNSC();

    explicit FrMaterialSurfaceNSC(FrBody* body);

    float rolling_friction;
    float spinning_friction;
    float cohesion;
    float dampingf;
    float compliance;
    float complianceT;
    float complianceRoll;
    float complianceSpin;

    void Print() const override;
  };

  std::shared_ptr<FrMaterialSurfaceNSC> MakeDefaultMaterialSurfaceNSC();

  namespace internal {
    std::shared_ptr<chrono::ChMaterialSurface> GetChronoMaterialNSC(const FrMaterialSurfaceNSC* params);
  }

} // end namespace frydom

#endif //FRYDOM_FRMATERIALSURFACENSC_H
