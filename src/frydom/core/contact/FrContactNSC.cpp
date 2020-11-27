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

#include "FrContactNSC.h"

#include "frydom/core/body/FrBody.h"

namespace frydom {

  FrContactParamsNSC::FrContactParamsNSC() :
      static_friction(0.6f),
      sliding_friction(0.6f),
      rolling_friction(0),
      spinning_friction(0),
      restitution(0),
      cohesion(0),
      dampingf(0),
      compliance(0),
      complianceT(0),
      complianceRoll(0),
      complianceSpin(0) {
  }

  FrContactParamsNSC::FrContactParamsNSC(FrBody *body) {

    auto ms = internal::GetChronoBody(body)->GetMaterialSurfaceNSC();

    static_friction = ms->GetSfriction();
    sliding_friction = ms->GetKfriction();
    rolling_friction = ms->GetRollingFriction();
    spinning_friction = ms->GetSpinningFriction();
    restitution = ms->GetRestitution();
    cohesion = ms->GetCohesion();
    dampingf = ms->GetDampingF();
    compliance = ms->GetCompliance();
    complianceT = ms->GetComplianceT();
    complianceRoll = ms->GetComplianceRolling();
    complianceSpin = ms->GetComplianceSpinning();
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


} // end namespace frydom
