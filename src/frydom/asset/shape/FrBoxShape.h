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
#ifndef FRYDOM_FRBOXSHAPE_H
#define FRYDOM_FRBOXSHAPE_H

#include <memory>
#include "frydom/asset/FrAssetOwner.h"

// forward declaration
namespace chrono {

  class ChBoxShape;

  class ChAsset;

}


namespace frydom {

  // forward declaration
  class FrBoxShape;

  namespace internal {

    std::shared_ptr<chrono::ChAsset> GetChronoAsset(std::shared_ptr<FrBoxShape> box);

    std::shared_ptr<chrono::ChAsset> GetChronoAsset(FrBoxShape *box);

  } // end namespace frydom::internal

  // forward declaration
  class Direction;

  class FrBoxShape {
   public:
    FrBoxShape(double xSize, double ySize, double zSize, const Position& relative_position, FRAME_CONVENTION fc);

    double xSize() const;

    double ySize() const;

    double zSize() const;

    void Translate(const Direction &direction) const;

   private:
    std::shared_ptr<chrono::ChBoxShape> m_box;

    friend std::shared_ptr<chrono::ChAsset> internal::GetChronoAsset(std::shared_ptr<FrBoxShape>);

    friend std::shared_ptr<chrono::ChAsset> internal::GetChronoAsset(FrBoxShape *);

  };



}  // end namespace frydom

#endif  // FRYDOM_FRBOXSHAPE_H
