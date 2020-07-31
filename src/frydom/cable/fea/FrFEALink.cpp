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

      //##CC
      std::cout << "debug : define log messages of the fea link" << std::endl;
      //##

      auto msg = NewMessage("State", "State message");

      //msg->AddField<double>("time", "s", "Current time of the simulation",
      //                      [this]() {
      //  UpdateCache();
      //  return GetOffshoreSystem()->GetTime();
      //});

      msg->AddField<double>("time", "s", "Current time of the simulation",
                            [this]() { return GetOffshoreSystem()->GetTime(); });

      // Node Position
      msg->AddField<Eigen::Matrix<double, 3, 1>>
          ("PositionOfNode2WRTNode1", "m",
           fmt::format("Node 2 position relatively to Node 1, in Node 1 reference frame in {}",
                       GetLogFC()),
           [this]() { return GetNode2PositionWRTNode1(GetLogFC()); });

      // Node Orientation
      msg->AddField<Eigen::Matrix<double, 3, 1>>
          ("OrientationOfNode2WRTNode1", "rad",
           fmt::format("Node 2 orientation relatively to Node 1, in Node 1 reference frame in {}", GetLogFC()),
           [this]() {
             double phi, theta, psi;
             GetNode2OrientationWRTNode1().GetCardanAngles_RADIANS(phi, theta, psi, GetLogFC());
             return Position(phi, theta, psi);
           });

      // Force
      msg->AddField<Eigen::Matrix<double, 3, 1>>
          ("LinkReactionForceOnBody1", "N",
           fmt::format("link reaction force applied at marker 1, expressed in marker 1 reference frame in {}", GetLogFC()),
           [this]() { return GetLinkReactionForceOnNode1(GetLogFC()); });
      msg->AddField<Eigen::Matrix<double, 3, 1>>
          ("LinkReactionForceOnBody2", "N",
           fmt::format("link reaction force applied at marker 2, expressed in marker 2 reference frame in {}", GetLogFC()),
           [this]() { return GetLinkReactionForceOnNode2(GetLogFC()); });

      // Torque
      msg->AddField<Eigen::Matrix<double, 3, 1>>
          ("LinkReactionTorqueOnBody1", "Nm",
           fmt::format("link reaction torque at Node 1, expressed in Node 1 reference frame in {}", GetLogFC()),
           [this]() { return GetLinkReactionTorqueOnNode1(GetLogFC()); });
      msg->AddField<Eigen::Matrix<double, 3, 1>>
          ("LinkReactionTorqueOnBody2", "Nm",
           fmt::format("link reaction torque at Node 2, expressed in Node 2 reference frame in {}", GetLogFC()),
           [this]() { return GetLinkReactionTorqueOnNode2(GetLogFC()); });

    }

    const Position FrFEALinkBase::GetNode2PositionWRTNode1(FRAME_CONVENTION fc) const {
      return c_frame2WRT1.GetPosition(fc);
    }

    const FrRotation FrFEALinkBase::GetNode2OrientationWRTNode1() const {
      return c_frame2WRT1.GetRotation();
    }

    const Force FrFEALinkBase::GetLinkReactionForceOnNode1(frydom::FRAME_CONVENTION fc) const {
      return -GetLinkReactionForceOnNode2(fc);
    }

    const Force FrFEALinkBase::GetLinkReactionForceOnNode2(frydom::FRAME_CONVENTION fc) const {
      auto force = c_generalizedForceOnNode2.GetForce();
      if (IsNED(fc)) internal::SwapFrameConvention<Force>(force);
      return force;
    }

    const Torque FrFEALinkBase::GetLinkReactionTorqueOnNode1(FRAME_CONVENTION fc) const {
      auto torque = c_generalizedForceOnNode1.GetTorque();
      if (IsNED(fc)) internal::SwapFrameConvention<Torque>(torque);
      return torque;
    }

    const Torque FrFEALinkBase::GetLinkReactionTorqueOnNode2(FRAME_CONVENTION fc) const {
      auto torque = c_generalizedForceOnNode2.GetTorque();
      if (IsNED(fc)) internal::SwapFrameConvention<Torque>(torque);
      return torque;
    }

    void FrFEALinkBase::UpdateCache() {

      // Relative frame
      c_frame2WRT1 = internal::ChFrame2FrFrame(frame2);
      c_frame1WRT2 = c_frame2WRT1.GetInverse();

      // Force and torque
      c_generalizedForceOnNode1.SetForce(internal::ChVectorToVector3d<Force>(Get_react_force()));
      c_generalizedForceOnNode1.SetTorque(internal::ChVectorToVector3d<Torque>(Get_react_torque()));

      c_generalizedForceOnNode2.SetForce(
          -c_frame1WRT2.ProjectVectorFrameInParent<Force>(c_generalizedForceOnNode1.GetForce(), NWU));
      c_generalizedForceOnNode2.SetTorque(
          -c_frame1WRT2.ProjectVectorFrameInParent(c_generalizedForceOnNode1.GetTorque(), NWU)
          + c_frame1WRT2.GetPosition(NWU).cross(c_generalizedForceOnNode2.GetForce()));

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
