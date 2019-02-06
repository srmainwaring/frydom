//
// Created by frongere on 31/01/19.
//

#ifndef FRYDOM_FRLINKBASE_H
#define FRYDOM_FRLINKBASE_H


#include <memory>
#include <vector>

#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrConvention.h"
#include "frydom/core/common/FrFrame.h"
#include "frydom/core/FrOffshoreSystem.h"

#include "frydom/core/common/FrPhysicsItem.h"


namespace chrono {
    class ChLink;
}

namespace frydom {

    // Forward declarations
    class FrNode_;
    class FrBody_;


    /// Pure abstract class for every FRyDoM constraints (FrLink_, FrConstraint_, FrActuator_)
    class FrLinkBase_ : public FrPhysicsItem_ {

    protected:

        std::shared_ptr<FrNode_> m_node1;   ///< the node on body 1 of the link
        std::shared_ptr<FrNode_> m_node2;   ///< the node on body 2 of the link

    public:
        FrLinkBase_(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);

        /// Tells if all constraints of this link are currently turned on or off by the user.
        virtual bool IsDisabled() const = 0;

        /// User can use this to enable/disable all the constraint of the link as desired.
        virtual void SetDisabled(bool disabled) = 0;

//        /// Tells if the link is broken, for excess of pulling/pushing.
//        virtual bool IsBroken() const = 0;
//
//        /// Set the 'broken' status vof this link.
//        virtual void SetBroken(bool broken) = 0;

        /// Tells if the link is currently active, in general,
        /// that is tells if it must be included into the system solver or not.
        /// This method cumulates the effect of various flags (so a link may
        /// be not active either because disabled, or broken, or not valid)
        virtual bool IsActive() const = 0;




        /// Returns the first node of the link
        std::shared_ptr<FrNode_> GetNode1();
        const std::shared_ptr<FrNode_> GetNode1() const;

        /// Returns the second node of the link
        std::shared_ptr<FrNode_> GetNode2();
        const std::shared_ptr<FrNode_> GetNode2() const;

        /// Returns the first body of the link
        FrBody_* GetBody1();
//        const FrBody_* GetBody1() const;

        /// Returns the second body of the link
        FrBody_* GetBody2();
//        const  FrBody_* GetBody2() const;

    protected:  // TODO : voir si on rend cela private


        friend void FrOffshoreSystem_::AddLink(std::shared_ptr<FrLinkBase_> link);
        virtual std::shared_ptr<chrono::ChLink> GetChronoLink() = 0;

        std::shared_ptr<chrono::ChBody> GetChronoBody1();
        std::shared_ptr<chrono::ChBody> GetChronoBody2();
    };



}  // end namespace frydom

#endif //FRYDOM_FRLINKBASE_H
