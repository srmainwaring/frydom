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

#include <iostream>
#include "FrContactSMC.h"

#include "frydom/core/body/FrBody.h"

namespace frydom {

FrContactParamsSMC::FrContactParamsSMC()
    : young_modulus(2e5),
      poisson_ratio(0.3f),
      static_friction(0.6f),
      sliding_friction(0.6f),
      restitution(0.4f),
      constant_adhesion(0),
      adhesionMultDMT(0) {}
//      kn(2e5),
//      kt(2e5),
//      gn(40),
//      gt(20){}

  FrContactParamsSMC::FrContactParamsSMC(FrBody *body) {

    auto ms = internal::GetChronoBody(body)->GetMaterialSurfaceSMC();

    static_friction = ms->GetSfriction();
    sliding_friction = ms->GetKfriction();
    young_modulus = ms->GetYoungModulus();
    poisson_ratio = ms->GetPoissonRatio();
    restitution = ms->GetRestitution();
    constant_adhesion = ms->GetAdhesion();
    adhesionMultDMT = ms->GetAdhesionMultDMT();

  }

  void FrContactParamsSMC::Print() const {
    std::cout << "SMC Contact parameters:" << std::endl;
    std::cout << "Young modulus   " << young_modulus << std::endl;
    std::cout << "Poisson ratio  " << poisson_ratio << std::endl;
    std::cout << "static friction   " << static_friction << std::endl;
    std::cout << "sliding friction  " << sliding_friction << std::endl;
    std::cout << "restitution       " << restitution << std::endl;
    std::cout << "constant adhesion          " << constant_adhesion << std::endl;
    std::cout << "adhesionMultDMT           " << adhesionMultDMT << std::endl;
  }

  std::shared_ptr<FrContactParamsSMC> MakeDefaultContactParamsSMC() {
    return std::make_shared<FrContactParamsSMC>();
  }

}
