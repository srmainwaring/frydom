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

#include "FrAsset.h"

namespace frydom {

  namespace internal {

    FrAssetBase::FrAssetBase(FrAsset *asset) : m_frydomAsset(asset) {}

    std::shared_ptr<FrAssetBase> GetChronoAsset(std::shared_ptr<FrAsset> asset) {
      return asset->m_chronoAsset;
    }

  } // end namespace frydom::internal


  FrAsset::FrAsset() {
    m_chronoAsset = std::make_shared<internal::FrAssetBase>(this);
  }

} // end namespace frydom
