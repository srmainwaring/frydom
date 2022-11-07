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

#ifndef FRYDOM_FRMATERIALSURFACESMC_H
#define FRYDOM_FRMATERIALSURFACESMC_H

#include <memory>
#include "FrMaterialSurface.h"

namespace frydom {

  class FrBody;

  class FrMaterialSurfaceSMC : public FrMaterialSurface {

    public:

    FrMaterialSurfaceSMC();

    explicit FrMaterialSurfaceSMC(FrBody* body);

    float young_modulus;      ///< Young's modulus (elastic modulus)
    float poisson_ratio;      ///< Poisson ratio
    float constant_adhesion;  ///< Constant adhesion force, when constant adhesion model is used
    float adhesionMultDMT;    ///< Adhesion multiplier used in DMT model.

//    float kn;  ///< user-specified normal stiffness coefficient
//    float kt;  ///< user-specified tangential stiffness coefficient
//    float gn;  ///< user-specified normal damping coefficient
//    float gt;  ///< user-specified tangential damping coefficient

    void Print() const override;

   //public:
   // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  std::shared_ptr<FrMaterialSurfaceSMC> MakeDefaultMaterialSurfaceSMC();

  namespace internal {
    std::shared_ptr<chrono::ChMaterialSurface> GetChronoMaterialSMC(const FrMaterialSurfaceSMC* params);
  }

} // end namespace frydom

#endif //FRYDOM_FRMATERIALSURFACESMC_H
