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


#ifndef FRYDOM_FRPHYSICSITEM_H
#define FRYDOM_FRPHYSICSITEM_H

#include "chrono/physics/ChPhysicsItem.h"

#include "FrObject.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/misc/FrColors.h"


namespace frydom {

    class FrPhysicsItem;


    namespace internal {

        struct FrPhysicsItemBase : public chrono::ChPhysicsItem {

            FrPhysicsItem *m_frydomPhysicsItem;

            /// Constructor.
            explicit FrPhysicsItemBase(FrPhysicsItem *item);

            void SetupInitial() override;

            void Update(bool update_assets) override;           // TODO : non nécessaire

            void Update(double time, bool update_assets) override;

            friend class FrPhysicsItem_;

        };

    }  // end namespace frydom::internal


    class FrOffshoreSystem;
    class FrTriangleMeshConnected;

    /**
     * \class FrPhysicsItem_
     * \brief Class for defining objects which are neither bodies nor links, for instance caterany lines.
     */
    class FrPhysicsItem: public FrObject {

    protected:
        std::shared_ptr<internal::FrPhysicsItemBase> m_chronoPhysicsItem;

        FrOffshoreSystem* m_system;

    public:

        FrPhysicsItem();

        FrOffshoreSystem* GetSystem();

        void SetName(const char name[]);

        std::string GetName() const;

        virtual void Update(double time) = 0;

        void AddMeshAsset(std::shared_ptr<frydom::FrTriangleMeshConnected> mesh);

        void SetColor(NAMED_COLOR colorName);

        void SetColor(const FrColor& color);

        virtual void SetupInitial();

        void Initialize() override {};
    protected:

        virtual std::shared_ptr<chrono::ChPhysicsItem> GetChronoPhysicsItem() const ;

    };

    /**
     * \class FrPrePhysicsItem_
     * \brief Class for defining physics items updated before bodies.
     */
    class FrPrePhysicsItem : public FrPhysicsItem{
    protected:
        friend void FrOffshoreSystem::AddPhysicsItem(std::shared_ptr<FrPrePhysicsItem>);
    };

    /**
     * \class FrMidPhysicsItem_
     * \brief Class for defining physics items updated after bodies but before links.
     */
    class FrMidPhysicsItem : public FrPhysicsItem{
    protected:
        friend void FrOffshoreSystem::AddPhysicsItem(std::shared_ptr<FrMidPhysicsItem>);
    };

    /**
     * \class FrPostPhysicsItem_
     * \brief Class for defining physics items updated after links.
     */
    class FrPostPhysicsItem : public FrPhysicsItem{
    protected:
        friend void FrOffshoreSystem::AddPhysicsItem(std::shared_ptr<FrPostPhysicsItem>);
    };

}  // end namespace frydom


#endif //FRYDOM_FRPHYSICSITEM_H
