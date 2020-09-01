//
// Created by frongere on 30/04/2020.
//

#include "FrFEALink.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"
#include "FrFEANode.h"

#include "frydom/core/FrOffshoreSystem.h"
#include "hermes/Messaging.h"
#include "frydom/logging/FrTypeNames.h"


namespace frydom {

  namespace internal {

    // ------------------------------------------------
    // FrFEALinkBase
    // ------------------------------------------------

    FrFEALinkBase::FrFEALinkBase(const std::string& name,
                                 FrOffshoreSystem* system,
                                 bool mc_x,
                                 bool mc_y,
                                 bool mc_z,
                                 bool mc_rx,
                                 bool mc_ry,
                                 bool mc_rz) :
        FrLoggable(name, TypeToString(this), system),
        chrono::ChLinkMateGeneric(mc_x,
                                    mc_y,
                                    mc_z,
                                    mc_rx,
                                    mc_ry,
                                    mc_rz) {
      LogThis(true);
    }

    void FrFEALinkBase::DefineLogMessages() {

      auto msg = NewMessage("State", "State message");

      // Time & Update

      msg->AddField<double>("time", "s", "Current time of the simulation",
                            [this]() {
        UpdateCache();
        return GetOffshoreSystem()->GetTime();
      });

      // Positions

      msg->AddField<Eigen::Matrix<double, 3, 1>>
          ("PositionOfNode1InWorld", "m",
           fmt::format("Node 1 position in world reference frame in {}", GetLogFC()),
           [this]() { return GetNode1PositionInWorld(GetLogFC()); });

      msg->AddField<Eigen::Matrix<double, 3, 1>>
          ("PositionOfNode2InWorld", "m",
           fmt::format("Node 2 position in world reference frame in {}", GetLogFC()),
           [this]() { return GetNode2PositionInWorld(GetLogFC()); });

      msg->AddField<Eigen::Matrix<double, 3, 1>>
          ("PositionOfNode2WRTNode1", "m",
           fmt::format("Node 2 position relative to the position of the Node 2 in world reference frame in {}", GetLogFC()),
           [this]() { return GetNodePosition1WRT2(GetLogFC()); });

      // TODO : force and torque logs

    }

    const Position FrFEALinkBase::GetNode1PositionInWorld(FRAME_CONVENTION fc) const {
      return c_frame1Abs.GetPosition(fc);
    }

    const Position FrFEALinkBase::GetNode2PositionInWorld(FRAME_CONVENTION fc) const {
      return c_frame2Abs.GetPosition(fc);
    }

    const Position FrFEALinkBase::GetNodePosition1WRT2(frydom::FRAME_CONVENTION fc) const {
      return GetNode1PositionInWorld(fc) - GetNode2PositionInWorld(fc);
    }

    void FrFEALinkBase::UpdateCache() {

      c_frame1Abs = internal::ChFrame2FrFrame(frame1 >> *GetBody1());
      c_frame2Abs = internal::ChFrame2FrFrame(frame2 >> *GetBody2());

    }

    // -----------------------------------------------------
    // FrFEANodeBodyDistance
    // -----------------------------------------------------

    FrFEANodeBodyDistance::FrFEANodeBodyDistance() : chrono::ChLinkDistance() {

    }

    void FrFEANodeBodyDistance::Initialize(std::shared_ptr<FrFEANodeBase> fea_node,
                                           std::shared_ptr<FrNode> body_node,
                                           const double &distance) {

      chrono::ChLinkDistance::Initialize(fea_node,
                                         internal::GetChronoBody(body_node->GetBody()),
                                         true,
                                         chrono::VNULL,
                                         internal::Vector3dToChVector(body_node->GetPositionInWorld(NWU)),
                                         false,
                                         distance
      );

    }


  }  // end namespace frydom::internal


}  // end namespace frydom
