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

#ifndef FRYDOM_FRCONTACTSMC_H
#define FRYDOM_FRCONTACTSMC_H

#include <memory>
#include "FrContact.h"

namespace frydom {

  class FrBody;

  class FrContactParamsSMC : public FrContactParams {

    public:

    FrContactParamsSMC();

    explicit FrContactParamsSMC(FrBody* body);

    float young_modulus;      ///< Young's modulus (elastic modulus)
    float poisson_ratio;      ///< Poisson ratio
    float constant_adhesion;  ///< Constant adhesion force, when constant adhesion model is used
    float adhesionMultDMT;    ///< Adhesion multiplier used in DMT model.

//    float kn;  ///< user-specified normal stiffness coefficient
//    float kt;  ///< user-specified tangential stiffness coefficient
//    float gn;  ///< user-specified normal damping coefficient
//    float gt;  ///< user-specified tangential damping coefficient

    void Print() const override;

  };

  std::shared_ptr<FrContactParamsSMC> MakeDefaultContactParamsSMC();

  namespace internal {
    std::shared_ptr<chrono::ChMaterialSurface> GetChronoMaterialSMC(const FrContactParamsSMC* params);
  }

} // end namespace frydom

#endif //FRYDOM_FRCONTACTSMC_H
