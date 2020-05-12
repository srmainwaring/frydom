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


#ifndef FRYDOM_FRCYLINDERSHAPE_H
#define FRYDOM_FRCYLINDERSHAPE_H

#include <memory>
#include "frydom/asset/FrAssetOwner.h"


// Forward declaration
namespace chrono {
  class ChAsset;

  class ChCylinderShape;
}  // end namespace chrono


namespace frydom {

  // Forward declaration
  class FrCylinderShape;

  namespace internal {
    std::shared_ptr<chrono::ChAsset> GetChronoAsset(std::shared_ptr<FrCylinderShape> cylinder);

    std::shared_ptr<chrono::ChAsset> GetChronoAsset(FrCylinderShape *cylinder);
  }  // end namespace frydom::internal


  class FrCylinderShape {

   public:

    FrCylinderShape(double radius, double height, const Position &relative_position, FRAME_CONVENTION fc);

    double radius() const;

    double height() const;

   private:

    std::shared_ptr<chrono::ChCylinderShape> m_cylinder;

    friend std::shared_ptr<chrono::ChAsset> internal::GetChronoAsset(std::shared_ptr<FrCylinderShape>);

    friend std::shared_ptr<chrono::ChAsset> internal::GetChronoAsset(FrCylinderShape *);

  };

}  // end namespace frydom

#endif  // FRYDOM_FRCYLINDERSHAPE_H
