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
#include "FrContact.h"

namespace frydom {


  // TODO: utiliser egalement le ChMaterialCompositeNSC pour faire interagir des materiaux differents

  // Forward declaration
  class FrBody;

  class FrContactParamsNSC: public FrContactParams {

    public:

    FrContactParamsNSC();

    explicit FrContactParamsNSC(FrBody* body);

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

  std::shared_ptr<FrContactParamsNSC> MakeDefaultContactParamsNSC();

  namespace internal {
    std::shared_ptr<chrono::ChMaterialSurface> GetChronoMaterialNSC(const FrContactParamsNSC& params);
  }

} // end namespace frydom

#endif //FRYDOM_FRCONTACTNSC_H
