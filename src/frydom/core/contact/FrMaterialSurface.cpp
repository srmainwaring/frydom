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

#include <typeinfo>
#include "chrono/physics/ChMaterialSurface.h"

#include "frydom/core/FrOffshoreSystem.h"

#include "FrMaterialSurface.h"
#include "FrMaterialSurfaceNSC.h"
#include "FrMaterialSurfaceSMC.h"

namespace frydom {

  FrMaterialSurface::FrMaterialSurface() :
    static_friction(0.6f),
    sliding_friction(0.6f),
    restitution(0) {}


  std::shared_ptr<chrono::ChMaterialSurface> internal::GetChronoMaterial(FrMaterialSurface* params) {
    if (typeid(*params) == typeid(FrMaterialSurfaceSMC)) {
      return internal::GetChronoMaterialSMC(dynamic_cast<FrMaterialSurfaceSMC*>(params));
    } else if (typeid(*params) == typeid(FrMaterialSurfaceNSC)) {
      return internal::GetChronoMaterialNSC(dynamic_cast<FrMaterialSurfaceNSC*>(params));
    } else {
      std::cerr << "Wrong material surface type" << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  std::shared_ptr<chrono::ChMaterialSurface> internal::GetChronoMaterial(FrMaterialSurfaceSMC* params) {
    return internal::GetChronoMaterialSMC(params);
  }

  std::shared_ptr<chrono::ChMaterialSurface> internal::GetChronoMaterial(FrMaterialSurfaceNSC* params) {
    return internal::GetChronoMaterialNSC(params);
  }

} // end namespace frydom

