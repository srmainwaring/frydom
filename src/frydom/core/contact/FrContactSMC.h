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

namespace frydom {

  class FrBody;

  struct FrContactParamsSMC {

    FrContactParamsSMC();

    explicit FrContactParamsSMC(FrBody* body);

    float young_modulus;      ///< Young's modulus (elastic modulus)
    float poisson_ratio;      ///< Poisson ratio
    float static_friction;    ///< Static coefficient of friction
    float sliding_friction;   ///< Kinetic coefficient of friction
    float restitution;        ///< Coefficient of restitution
    float constant_adhesion;  ///< Constant adhesion force, when constant adhesion model is used
    float adhesionMultDMT;    ///< Adhesion multiplier used in DMT model.

//    float kn;  ///< user-specified normal stiffness coefficient
//    float kt;  ///< user-specified tangential stiffness coefficient
//    float gn;  ///< user-specified normal damping coefficient
//    float gt;  ///< user-specified tangential damping coefficient

    void Print() const;

  };

  std::shared_ptr<FrContactParamsSMC> MakeDefaultContactParamsSMC();

}
#endif //FRYDOM_FRCONTACTSMC_H
