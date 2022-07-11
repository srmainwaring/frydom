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
#include <memory>

#include "FrContactNSC.h"

#include "frydom/core/body/FrBody.h"

namespace frydom {

  FrContactParamsNSC::FrContactParamsNSC() :
      FrContactParams(),
      rolling_friction(0),
      spinning_friction(0),
      cohesion(0),
      dampingf(0),
      compliance(0),
      complianceT(0),
      complianceRoll(0),
      complianceSpin(0) {
  }

  FrContactParamsNSC::FrContactParamsNSC(FrBody *body) {

    auto ms = internal::GetChronoBody(body)->GetCollisionModel()->GetShape(0)->GetMaterial();
    auto ms_nsc = std::dynamic_pointer_cast<chrono::ChMaterialSurfaceNSC>(ms);

    static_friction = ms_nsc->GetSfriction();
    sliding_friction = ms_nsc->GetKfriction();
    rolling_friction = ms_nsc->GetRollingFriction();
    spinning_friction = ms_nsc->GetSpinningFriction();
    restitution = ms_nsc->GetRestitution();
    cohesion = ms_nsc->GetCohesion();
    dampingf = ms_nsc->GetDampingF();
    compliance = ms_nsc->GetCompliance();
    complianceT = ms_nsc->GetComplianceT();
    complianceRoll = ms_nsc->GetComplianceRolling();
    complianceSpin = ms_nsc->GetComplianceSpinning();
  }

  void FrContactParamsNSC::Print() const {
    std::cout << "NSC Contact parameters:" << std::endl;
    std::cout << "static friction   " << static_friction << std::endl;
    std::cout << "sliding friction  " << sliding_friction << std::endl;
    std::cout << "rolling friction  " << rolling_friction << std::endl;
    std::cout << "spinning friction " << spinning_friction << std::endl;
    std::cout << "restitution       " << restitution << std::endl;
    std::cout << "cohesion          " << cohesion << std::endl;
    std::cout << "damping           " << dampingf << std::endl;
    std::cout << "compliance        " << compliance << std::endl;
    std::cout << "complianceT       " << complianceT << std::endl;
    std::cout << "compliance Roll   " << complianceRoll << std::endl;
    std::cout << "compliance spin   " << complianceSpin << std::endl;
  }

  std::shared_ptr<FrContactParamsNSC> MakeDefaultContactParamsNSC() {
    return std::make_shared<FrContactParamsNSC>();
  }

  std::shared_ptr<chrono::ChMaterialSurface> internal::GetChronoMaterialNSC(const FrContactParamsNSC& params) {

    // FIXME (CC) : regarder pour faire de la composition de ChMaterialSurface avec FrContactParams

    auto chrono_mat = std::make_shared<chrono::ChMaterialSurfaceNSC>();

    chrono_mat->SetSfriction(params.static_friction);
    chrono_mat->SetKfriction(params.sliding_friction);
    chrono_mat->SetRollingFriction(params.rolling_friction);
    chrono_mat->SetSpinningFriction(params.spinning_friction);
    chrono_mat->SetRestitution(params.restitution);
    chrono_mat->SetCohesion(params.cohesion);
    chrono_mat->SetDampingF(params.dampingf);
    chrono_mat->SetCompliance(params.compliance);
    chrono_mat->SetComplianceT(params.complianceT);
    chrono_mat->SetComplianceRolling(params.complianceRoll);
    chrono_mat->SetComplianceSpinning(params.complianceSpin);

    return chrono_mat;

  }


} // end namespace frydom
