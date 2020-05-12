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

#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/misc/FrColors.h"

#include "frydom/core/common/FrTreeNode.h"


namespace frydom {

  // Forward declaration
  class FrPhysicsItem;


  namespace internal {

    struct FrPhysicsItemBase : public chrono::ChPhysicsItem {

      FrPhysicsItem *m_frydomPhysicsItem;

      explicit FrPhysicsItemBase(FrPhysicsItem *item);

      void SetupInitial() override;

      void Update(double time, bool update_assets) override;

    };

    std::shared_ptr<FrPhysicsItemBase> GetChronoPhysicsItem(std::shared_ptr<FrPhysicsItem> item);

    std::shared_ptr<FrPhysicsItemBase> GetChronoPhysicsItem(FrPhysicsItem *item);

  }  // end namespace frydom::internal


  class FrOffshoreSystem;

  class FrTriangleMeshConnected;

  class FrAsset;

  /**
   * \class FrPhysicsItem
   * \brief Class for defining objects which are neither bodies nor links, for instance caterany lines.
   */
  class FrPhysicsItem : public FrObject {

   public:

    FrPhysicsItem();

    /// Check if the force is active
    bool IsActive() const;

    /// Activate or deactivate
    void SetActive(bool active);

    /// Return true if the physics item is included in the static analysis
    virtual bool IncludedInStaticAnalysis() const { return true; }

    void Update(double time);

    virtual void SetupInitial();

    void Initialize() override {};

   protected:
    /// Get the shared pointer to the chrono related physics item
    /// \return Chrono related physics item
    virtual std::shared_ptr<internal::FrPhysicsItemBase> GetChronoPhysicsItem() const;

   private:

    /// Virtual function to allow updating the child object from the solver
    /// \param time Current time of the simulation from beginning, in seconds
    virtual void Compute(double time) = 0;

   protected:

    std::shared_ptr<internal::FrPhysicsItemBase> m_chronoPhysicsItem; ///> pointer to the related chrono physics item

    bool m_isActive = true;  ///< boolean to check if the physics item is active
    ///< if it's not the case, it is not updated during the simulation


    friend std::shared_ptr<internal::FrPhysicsItemBase> internal::GetChronoPhysicsItem(std::shared_ptr<FrPhysicsItem>);

    friend std::shared_ptr<internal::FrPhysicsItemBase> internal::GetChronoPhysicsItem(FrPhysicsItem *);

  };

}  // end namespace frydom


#endif //FRYDOM_FRPHYSICSITEM_H
