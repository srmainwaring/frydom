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

#ifndef FRYDOM_FRMATERIALSURFACE_H
#define FRYDOM_FRMATERIALSURFACE_H

namespace chrono {
  class ChMaterialSurface;
}

namespace frydom {

  // Forward declaration
  class FrMaterialSurfaceSMC;
  class FrMaterialSurfaceNSC;
  class FrOffshoreSystem;

  class FrMaterialSurface {

    public:

    FrMaterialSurface();

    float static_friction;
    float sliding_friction;
    float restitution;

    virtual void Print() const = 0;

   //public:
   // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };


  namespace internal {
    std::shared_ptr<chrono::ChMaterialSurface> GetChronoMaterial(FrMaterialSurface* params);
    std::shared_ptr<chrono::ChMaterialSurface> GetChronoMaterial(FrMaterialSurfaceSMC* params);
    std::shared_ptr<chrono::ChMaterialSurface> GetChronoMaterial(FrMaterialSurfaceNSC* params);
  }


} // end namespace frydom


#endif //FRYDOM_FRMATERIALSURFACE_H
