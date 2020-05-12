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


#include "FrPhysicsItem.h"


namespace frydom {

  namespace internal {


    FrPhysicsItemBase::FrPhysicsItemBase(FrPhysicsItem *item) : m_frydomPhysicsItem(item) {}

    void FrPhysicsItemBase::SetupInitial() {}

    void FrPhysicsItemBase::Update(double time, bool update_assets) {
      m_frydomPhysicsItem->Update(time);
      ChPhysicsItem::Update(time, update_assets);
    }

    std::shared_ptr<FrPhysicsItemBase> GetChronoPhysicsItem(std::shared_ptr<FrPhysicsItem> item) {
      return item->m_chronoPhysicsItem;
    }

    std::shared_ptr<FrPhysicsItemBase> GetChronoPhysicsItem(FrPhysicsItem* item) {
      return item->m_chronoPhysicsItem;
    }

  }  // end namespace frydom::internal


  FrPhysicsItem::FrPhysicsItem() : m_chronoPhysicsItem(std::make_shared<internal::FrPhysicsItemBase>(this)) {}

  bool FrPhysicsItem::IsActive() const {
    return m_isActive;
  }

  void FrPhysicsItem::SetActive(bool active) {
    m_isActive = active;
  }

  std::shared_ptr<internal::FrPhysicsItemBase> FrPhysicsItem::GetChronoPhysicsItem() const {
    return m_chronoPhysicsItem;
  }

  void FrPhysicsItem::SetupInitial() {
    m_chronoPhysicsItem->SetupInitial();
    Initialize();
  }

  void FrPhysicsItem::Update(double time) {
    if (IsActive())
      Compute(time);
  }


}  // end namespace frydom
