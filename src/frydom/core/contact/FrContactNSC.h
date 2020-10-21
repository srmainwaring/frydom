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

#ifndef FRYDOM_FRCONTACTNSC_H
#define FRYDOM_FRCONTACTNSC_H

#include <memory>

namespace frydom {

  // TODO: utiliser egalement le ChMaterialCompositeNSC pour faire interagir des materiaux differents

  // Forward declaration
  class FrBody;

  struct FrContactParamsNSC {

    FrContactParamsNSC();

    explicit FrContactParamsNSC(FrBody* body);

    float static_friction;
    float sliding_friction;
    float rolling_friction;
    float spinning_friction;
    float restitution;
    float cohesion;
    float dampingf;
    float compliance;
    float complianceT;
    float complianceRoll;
    float complianceSpin;

    void Print() const;
  };

  std::shared_ptr<FrContactParamsNSC> MakeDefaultContactParamsNSC();

} // end namespace frydom

#endif //FRYDOM_FRCONTACTNSC_H
