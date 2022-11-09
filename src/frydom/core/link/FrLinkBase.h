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


#ifndef FRYDOM_FRLINKBASE_H
#define FRYDOM_FRLINKBASE_H


#include <memory>
#include <vector>

#include "frydom/core/math/FrVector.h"
#include "frydom/core/math/FrTorsor.h"
#include "frydom/core/common/FrConvention.h"
#include "frydom/core/common/FrFrame.h"
#include "frydom/core/common/FrTreeNode.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/asset/FrAssetOwner.h"
#include "frydom/logging/FrLoggable.h"


namespace chrono {
  class ChLink;
}

namespace frydom {

  // Forward declaration
  class FrBodyBase;

  namespace internal {

    std::shared_ptr<FrBodyBase> GetChronoBody1(std::shared_ptr<FrLinkBase> link);

    std::shared_ptr<FrBodyBase> GetChronoBody1(FrLinkBase *link);

    std::shared_ptr<FrBodyBase> GetChronoBody2(std::shared_ptr<FrLinkBase> link);

    std::shared_ptr<FrBodyBase> GetChronoBody2(FrLinkBase *link);

  }  // end namespace frydom::internal


  enum ACTUATOR_CONTROL {
    POSITION,
    VELOCITY,
    FORCE
  };

  // Forward declarations
  class FrNode;

  class FrBody;


  /**
   * \class FrLinkBase
   * \brief Pure abstract class for every FRyDoM constraints (FrLink, FrConstraint_, FrActuator_).
   */
  // FIXME :: pass the FrLoggable from FrLinkBase to FrLink ( don't forget constraints and actuators)
  class FrLinkBase : public FrObject {

   protected:

    std::shared_ptr<FrNode> m_node1;   ///< the node on body 1 of the link
    std::shared_ptr<FrNode> m_node2;   ///< the node on body 2 of the link

   public:
    virtual ~FrLinkBase();

    FrLinkBase(const std::shared_ptr<FrNode> &node1,
               const std::shared_ptr<FrNode> &node2);

    /// Tells if all constraints of this link are currently turned on or off by the user.
    virtual bool IsDisabled() const = 0;

    /// User can use this to enable/disable all the constraint of the link as desired.
    virtual void SetDisabled(bool disabled) = 0;

    /// Tells if the link is currently active, in general,
    /// that is tells if it must be included into the system solver or not.
    /// This method cumulates the effect of various flags (so a link may
    /// be not active either because disabled, or broken, or not valid)
    virtual bool IsActive() const = 0;

    /// Return true if the link is included in the static analysis
    bool IncludedInStaticAnalysis() const { return true; }

    /// Returns the first node of the link
    std::shared_ptr<FrNode> GetNode1();

    const std::shared_ptr<FrNode> GetNode1() const;

    /// Returns the second node of the link
    std::shared_ptr<FrNode> GetNode2();

    const std::shared_ptr<FrNode> GetNode2() const;

    /// Returns the first body of the link
    FrBody *GetBody1();

    /// Returns the second body of the link
    FrBody *GetBody2();

    void StepFinalize() override {}


    friend std::shared_ptr<internal::FrBodyBase> internal::GetChronoBody1(std::shared_ptr<FrLinkBase>);

    friend std::shared_ptr<internal::FrBodyBase> internal::GetChronoBody1(FrLinkBase *);

    friend std::shared_ptr<internal::FrBodyBase> internal::GetChronoBody2(std::shared_ptr<FrLinkBase>);

    friend std::shared_ptr<internal::FrBodyBase> internal::GetChronoBody2(FrLinkBase *);


  };


}  // end namespace frydom

#endif //FRYDOM_FRLINKBASE_H
