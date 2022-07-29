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

#include "FrContact.h"
#include "FrContactNSC.h"
#include "FrContactSMC.h"

namespace frydom {

  FrContactParams::FrContactParams() :
    static_friction(0.6f),
    sliding_friction(0.6f),
    restitution(0) {}


  std::shared_ptr<chrono::ChMaterialSurface> internal::GetChronoMaterial(FrContactParams* params) {
    if (typeid(params) == typeid(FrContactParamsSMC)) {
      return internal::GetChronoMaterialSMC(dynamic_cast<FrContactParamsSMC*>(params));
    } else if (typeid(params) == typeid(FrContactParamsNSC)) {
      return internal::GetChronoMaterialSMC(dynamic_cast<FrContactParamsSMC*>(params));
    };
  }

  std::shared_ptr<chrono::ChMaterialSurface> internal::GetChronoMaterial(FrContactParamsSMC* params) {
    return internal::GetChronoMaterialSMC(params);
  }

  std::shared_ptr<chrono::ChMaterialSurface> internal::GetChronoMaterial(FrContactParamsNSC* params) {
    return internal::GetChronoMaterialNSC(params);
  }

} // end namespace frydom

