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


#ifndef FRYDOM_FRSPHERESHAPE_H
#define FRYDOM_FRSPHERESHAPE_H

#include <memory>
#include "frydom/asset/FrAssetOwner.h"

namespace chrono {
  class ChAsset;

  class ChSphereShape;
}  // end namespace chrono

namespace frydom {


  // forward declaration
  class FrSphereShape;

  namespace internal {

    std::shared_ptr<chrono::ChAsset> GetChronoAsset(std::shared_ptr<FrSphereShape> sphere);

    std::shared_ptr<chrono::ChAsset> GetChronoAsset(FrSphereShape *sphere);

  } // end namespace frydom::internal


  class FrSphereShape {

   public:
    FrSphereShape(double radius, const Position& relative_position, FRAME_CONVENTION fc);

    double radius() const;

   private:
    std::shared_ptr<chrono::ChSphereShape> m_sphere;

    friend std::shared_ptr<chrono::ChAsset> internal::GetChronoAsset(std::shared_ptr<FrSphereShape>);

    friend std::shared_ptr<chrono::ChAsset> internal::GetChronoAsset(FrSphereShape *);

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

}  // end namespace frydom

#endif  // FRYDOM_FRSPHERESHAPE_H
