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

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChColorAsset.h"

#include "frydom/asset/FrAsset.h"
#include "frydom/mesh/FrTriangleMeshConnected.h"

namespace frydom {

    namespace internal {


        FrPhysicsItemBase::FrPhysicsItemBase(FrPhysicsItem *item) : m_frydomPhysicsItem(item) {}

        void FrPhysicsItemBase::SetupInitial() {
        }

        void FrPhysicsItemBase::Update(bool update_assets) {
            this->Update(ChTime, update_assets);
        }

        void FrPhysicsItemBase::Update(double time, bool update_assets) {
            m_frydomPhysicsItem->Update(time);
            ChPhysicsItem::Update(time, update_assets);
        }

    }  // end namespace frydom::internal



    FrPhysicsItem::FrPhysicsItem() {
        m_chronoPhysicsItem = std::make_shared<internal::FrPhysicsItemBase>(this);
    };

    FrOffshoreSystem* FrPhysicsItem::GetSystem() {
        return m_system;
    }

    void FrPhysicsItem::SetName(const char *name) {
        m_chronoPhysicsItem->SetName(name);
    }

    std::string FrPhysicsItem::GetName() const {
        return m_chronoPhysicsItem->GetNameString();
    }

    std::shared_ptr<chrono::ChPhysicsItem> FrPhysicsItem::GetChronoPhysicsItem() const {
        return m_chronoPhysicsItem;
    }

    void FrPhysicsItem::AddMeshAsset(std::shared_ptr<frydom::FrTriangleMeshConnected> mesh) {
        auto shape = std::make_shared<chrono::ChTriangleMeshShape>();
        shape->SetMesh(*mesh);
        m_chronoPhysicsItem->AddAsset(shape);
    }

    void FrPhysicsItem::SetColor(NAMED_COLOR colorName) {
        SetColor(FrColor(colorName));
    }

    void FrPhysicsItem::SetColor(const FrColor& color) {
        auto colorAsset = std::make_shared<chrono::ChColorAsset>(
                chrono::ChColor(color.R, color.G, color.B));
        m_chronoPhysicsItem->AddAsset(colorAsset);
    }

    void FrPhysicsItem::SetupInitial() {
        m_chronoPhysicsItem->SetupInitial();
        Initialize();
    }

    void FrPhysicsItem::AddAsset(std::shared_ptr<FrAsset> asset) {
        
//        m_assets.push_back(asset);
        m_chronoPhysicsItem->AddAsset(asset->GetChronoAsset());

    }

}  // end namespace frydom
