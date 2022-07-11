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
    : FrContactParams(),
      young_modulus(2e5),
      poisson_ratio(0.3f),
      constant_adhesion(0),
      adhesionMultDMT(0) {}
//      kn(2e5),
//      kt(2e5),
//      gn(40),
//      gt(20){}

  FrContactParamsSMC::FrContactParamsSMC(FrBody *body) {

    auto ms = internal::GetChronoBody(body)->GetCollisionModel()->GetShape(0)->GetMaterial();
    auto ms_smc = std::dynamic_pointer_cast<chrono::ChMaterialSurfaceSMC>(ms);

    static_friction = ms_smc->GetSfriction();
    sliding_friction = ms_smc->GetKfriction();
    young_modulus = ms_smc->GetYoungModulus();
    poisson_ratio = ms_smc->GetPoissonRatio();
    restitution = ms_smc->GetRestitution();
    constant_adhesion = ms_smc->GetAdhesion();
    adhesionMultDMT = ms_smc->GetAdhesionMultDMT();

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

  std::shared_ptr<chrono::ChMaterialSurface> internal::GetChronoMaterialSMC(const FrContactParamsSMC& params){

    auto chrono_mat = std::make_shared<chrono::ChMaterialSurfaceSMC>();

    chrono_mat->SetYoungModulus(params.young_modulus);
    chrono_mat->SetPoissonRatio(params.poisson_ratio);
    chrono_mat->SetSfriction(params.static_friction);
    chrono_mat->SetKfriction(params.sliding_friction);
    chrono_mat->SetRestitution(params.restitution);
    chrono_mat->SetAdhesion(params.constant_adhesion);
    chrono_mat->SetAdhesionMultDMT(params.adhesionMultDMT);

    return chrono_mat;
  }

}
