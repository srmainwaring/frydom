//
// Created by frongere on 30/04/2020.
//

#ifndef FRYDOM_FRFEALINK_H
#define FRYDOM_FRFEALINK_H

#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChLinkDistance.h>

#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrRotation.h"
#include "frydom/core/force/FrForce.h"
#include "frydom/logging/FrLoggable.h"


namespace frydom {

  class FrOffshoreSystem;

  // Forward declaration
  class FrNode;

  namespace internal {

    class FrFEALinkBase : public chrono::ChLinkMateGeneric, public FrLoggable<FrOffshoreSystem> { // TODO: en faire un loggable ???

     public:

      FrFEALinkBase(const std::string& name,
                    FrOffshoreSystem* system,
                    bool mc_x = true,
                    bool mc_y = true,
                    bool mc_z = true,
                    bool mc_rx = true,
                    bool mc_ry = true,
                    bool mc_rz = true);

      inline FrOffshoreSystem *GetOffshoreSystem() const {
        return GetParent();
      }

      const std::string &GetName() const { return FrLoggable<FrOffshoreSystem>::GetName(); }

      const Position GetNode2PositionWRTNode1(FRAME_CONVENTION fc) const;

      const FrRotation GetNode2OrientationWRTNode1() const;

      const Force GetLinkReactionForceOnNode1(FRAME_CONVENTION fc) const;

      const Force GetLinkReactionForceOnNode2(FRAME_CONVENTION fc) const;

      const Torque GetLinkReactionTorqueOnNode1(FRAME_CONVENTION fc) const;

      const Torque GetLinkReactionTorqueOnNode2(FRAME_CONVENTION fc) const;

      void UpdateCache();

     protected:

      void DefineLogMessages() override;

     private:

      FrFrame c_frame1WRT2;
      FrFrame c_frame2WRT1;

      GeneralizedForce c_generalizedForceOnNode2;
      GeneralizedForce c_generalizedForceOnNode1;

    };


    // Forward declaration
    class FrFEANodeBase;

    class FrFEANodeBodyDistance : public chrono::ChLinkDistance {
     public:
      FrFEANodeBodyDistance();

      void Initialize(std::shared_ptr<FrFEANodeBase> fea_node,
                      std::shared_ptr<FrNode> body_node,
                      const double &distance);


    };

//    class FrFEANodeNodePosition : public





  }  // end namspace frydom::internal


//  class FrFEALink {
//
//   public:
//    FrFEALink();
//
//   private:
//    std::shared_ptr<internal::FrFEALinkBase> m_chrono_link;
//
//
//  };

}  // end namespace frydom



#endif //FRYDOM_FRFEALINK_H
