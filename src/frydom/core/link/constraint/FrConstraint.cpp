//
// Created by lletourn on 22/05/19.
//

#include "FrConstraint.h"

#include "FrCGeometrical.h"
#include "frydom/core/common/FrNode.h"
#include "frydom/core/common/FrFrame.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/logging/FrTypeNames.h"


namespace frydom {


  FrConstraint::FrConstraint(const std::string &name,
                             const std::string &type_name,
                             FrOffshoreSystem *system,
                             const std::shared_ptr<FrNode> &node1,
                             const std::shared_ptr<FrNode> &node2) :
      FrLoggable(name, type_name, system),
      FrLinkBase(node1, node2) {
  }

  FrFrame FrConstraint::GetConstraintReferenceFrameInWorld() const {
    chrono::ChFrame<> chFrame(m_chronoConstraint->GetLinkAbsoluteCoords());
    return internal::ChFrame2FrFrame(chFrame);
  }

  FrFrame FrConstraint::GetConstraintReferenceFrameInBody1() const {
    chrono::ChFrame<> chFrame(m_chronoConstraint->GetLinkRelativeCoords());
    return internal::ChFrame2FrFrame(chFrame);
  }

  Force FrConstraint::GetForceInConstraint(FRAME_CONVENTION fc) const {
    auto force = internal::ChVectorToVector3d<Force>(internal::GetChronoConstraint(this)->Get_react_force());
    if (IsNED(fc)) force = internal::SwapFrameConvention(force);
    return force;
  }

  Torque FrConstraint::GetTorqueInConstraint(FRAME_CONVENTION fc) const {
    auto torque = internal::ChVectorToVector3d<Torque>(internal::GetChronoConstraint(this)->Get_react_torque());
    if (IsNED(fc)) torque = internal::SwapFrameConvention(torque);
    return torque;
  }

  Force FrConstraint::GetForceInBody1(FRAME_CONVENTION fc) const {
    return GetConstraintReferenceFrameInBody1().ProjectVectorFrameInParent(GetForceInConstraint(fc), fc);
  }

  Torque FrConstraint::GetTorqueInBody1AtCOG(FRAME_CONVENTION fc) const {
    auto force = GetForceInBody1(fc);
    auto torque = GetConstraintReferenceFrameInBody1().ProjectVectorFrameInParent(GetTorqueInConstraint(fc), fc);
    auto pos = GetConstraintReferenceFrameInBody1().GetPosition(fc);
    return torque + pos.cross(force);
  }

  Force FrConstraint::GetForceInWorld(FRAME_CONVENTION fc) const {
    return GetConstraintReferenceFrameInWorld().ProjectVectorFrameInParent(GetForceInConstraint(fc), fc);
//        return m_node1->ProjectVectorInWorld(GetForceInConstraint(fc), fc);
  }

  Torque FrConstraint::GetTorqueInWorldAtConstraint(FRAME_CONVENTION fc) const {
    return GetConstraintReferenceFrameInWorld().ProjectVectorFrameInParent(GetTorqueInConstraint(fc), fc);
//        return m_node1->ProjectVectorInWorld(GetTorqueInConstraint(fc), fc);
  }

  bool FrConstraint::IsDisabled() const {
    return m_chronoConstraint->IsDisabled();
  }

  void FrConstraint::SetDisabled(bool disabled) {
    m_chronoConstraint->SetDisabled(disabled);
  }

  bool FrConstraint::IsActive() const {
    return m_chronoConstraint->IsActive();
  }

  void FrConstraint::DefineLogMessages() {

    auto msg = NewMessage("State", "State message");

    msg->AddField<double>("time", "s", "Current time of the simulation",
                          [this]() { return GetSystem()->GetTime(); });

    // Constraint position and orientation
    msg->AddField<Eigen::Matrix<double, 3, 1 >>
        ("ConstraintPositionInWorld", "m",
         fmt::format(
             "Constraint reference frame position, relatively to the world reference frame, in {}",
             GetLogFC()),
         [this]() {
           return GetConstraintReferenceFrameInWorld().GetPosition(
               GetLogFC());
         });
    msg->AddField<Eigen::Matrix<double, 3, 1 >>
        ("ConstraintOrientationInWorld", "deg",
         fmt::format(
             "Constraint reference frame orientation, relatively to the world reference frame, in {}",
             GetLogFC()),
         [this]() {
           double phi, theta, psi;
           GetConstraintReferenceFrameInWorld().GetRotation().GetCardanAngles_DEGREES(
               phi, theta, psi, GetLogFC());
           return Position(phi, theta, psi);
         });

    // Constraint reaction force and torque
    msg->AddField<Eigen::Matrix<double, 3, 1 >>
        ("GetForceInWorld", "N", fmt::format(
            "Constraint reaction force in world reference frame, in {}",
            GetLogFC()),
         [this]() { return GetForceInWorld(GetLogFC()); });
    msg->AddField<Eigen::Matrix<double, 3, 1 >>
        ("GetTorqueInWorldAtConstraint", "Nm",
         fmt::format(
             "Constraint reaction torque at constraint reference frame origin, in world reference frame, in {}",
             GetLogFC()),
         [this]() { return GetTorqueInWorldAtConstraint(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1 >>
        ("GetForceInBody1", "N",
         fmt::format(
             "Constraint reaction force in first body reference frame, in {}",
             GetLogFC()),
         [this]() { return GetForceInBody1(GetLogFC()); });
    msg->AddField<Eigen::Matrix<double, 3, 1 >>
        ("GetTorqueInBody1AtCOG", "Nm",
         fmt::format(
             "Constraint reaction torque at COG, in first body reference frame, in {}",
             GetLogFC()),
         [this]() { return GetTorqueInBody1AtCOG(GetLogFC()); });
  }

  namespace internal {

    std::shared_ptr<chrono::ChLink> GetChronoConstraint(std::shared_ptr<FrConstraint> constraint) {
      return constraint->m_chronoConstraint;
    }

    std::shared_ptr<chrono::ChLink> GetChronoConstraint(const FrConstraint *constraint) {
      return constraint->m_chronoConstraint;
    }

  }  // end namespace frydom::internal

  //------------------------------------------------------------------------------------------------------------------

  FrConstraintParallel::FrConstraintParallel(const std::string &name,
                                             FrOffshoreSystem *system,
                                             const std::shared_ptr<FrCAxis> &axis1,
                                             const std::shared_ptr<FrCAxis> &axis2) :
      FrConstraint(name, TypeToString(this), system, axis1->GetNode(), axis2->GetNode()),
      m_axis1(axis1), m_axis2(axis2) {

    m_chronoConstraint = std::make_shared<chrono::ChLinkMateParallel>();
  }

  void FrConstraintParallel::Initialize() {

    auto chPos1 = internal::Vector3dToChVector(m_axis1->GetOriginInWorld(NWU));
    auto chPos2 = internal::Vector3dToChVector(m_axis2->GetOriginInWorld(NWU));
    auto chDir1 = internal::Vector3dToChVector(m_axis1->GetDirectionInWorld(NWU));
    auto chDir2 = internal::Vector3dToChVector(m_axis2->GetDirectionInWorld(NWU));

    auto chrono_item =
        std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(internal::GetChronoConstraint(this));
    chrono_item->Initialize(internal::GetChronoBody2(this),
                            internal::GetChronoBody1(this),
                            false,
                            chPos2,
                            chPos1,
                            chDir2,
                            chDir1);

//    GetSystem()->GetLogManager()->Add(this);

  }

  std::shared_ptr<FrConstraintParallel>
  make_constraint_parallel(const std::string &name,
                           FrOffshoreSystem *system,
                           const std::shared_ptr<FrCAxis> &axis1,
                           const std::shared_ptr<FrCAxis> &axis2) {

    auto constraint = std::make_shared<FrConstraintParallel>(name, system, axis1, axis2);
    system->Add(constraint);

    return constraint;

  }

  //------------------------------------------------------------------------------------------------------------------

  FrConstraintPerpendicular::FrConstraintPerpendicular(const std::string &name,
                                                       FrOffshoreSystem *system,
                                                       const std::shared_ptr<FrCAxis> &axis1,
                                                       const std::shared_ptr<FrCAxis> &axis2) :
      FrConstraint(name, TypeToString(this), system, axis1->GetNode(), axis2->GetNode()),
      m_axis1(axis1), m_axis2(axis2) {

    m_chronoConstraint = std::make_shared<chrono::ChLinkMateOrthogonal>();
  }

  void FrConstraintPerpendicular::Initialize() {

    auto chPos1 = internal::Vector3dToChVector(m_axis1->GetOriginInWorld(NWU));
    auto chDir1 = internal::Vector3dToChVector(m_axis1->GetDirectionInWorld(NWU));
    auto chPos2 = internal::Vector3dToChVector(m_axis2->GetOriginInWorld(NWU));
    auto chDir2 = internal::Vector3dToChVector(m_axis2->GetDirectionInWorld(NWU));

    auto chrono_item =
        std::dynamic_pointer_cast<chrono::ChLinkMateOrthogonal>(internal::GetChronoConstraint(this));

    chrono_item->Initialize(internal::GetChronoBody2(this),
                            internal::GetChronoBody1(this),
                            false,
                            chPos2,
                            chPos1,
                            chDir2,
                            chDir1);

  }

  std::shared_ptr<FrConstraintPerpendicular>
  make_constraint_perpendicular(const std::string &name,
                                FrOffshoreSystem *system,
                                const std::shared_ptr<FrCAxis> &axis1,
                                const std::shared_ptr<FrCAxis> &axis2) {

    auto constraint = std::make_shared<FrConstraintPerpendicular>(name, system, axis1, axis2);
    system->Add(constraint);

    return constraint;
  }

  //------------------------------------------------------------------------------------------------------------------

  FrConstraintPlaneOnPlane::FrConstraintPlaneOnPlane(const std::string &name,
                                                     FrOffshoreSystem *system,
                                                     const std::shared_ptr<FrCPlane> &plane1,
                                                     const std::shared_ptr<FrCPlane> &plane2,
                                                     bool flipped,
                                                     double distance) :
      FrConstraint(name, TypeToString(this), system, plane1->GetNode(), plane2->GetNode()),
      m_plane1(plane1), m_plane2(plane2) {

    m_chronoConstraint = std::make_shared<chrono::ChLinkMatePlane>();
    SetFlipped(flipped);
    SetDistance(distance);
  }

  void FrConstraintPlaneOnPlane::Initialize() {

    auto chPos1 = internal::Vector3dToChVector(m_plane1->GetOriginInWorld(NWU));
    auto chDir1 = internal::Vector3dToChVector(m_plane1->GetNormaleInWorld(NWU));
    auto chPos2 = internal::Vector3dToChVector(m_plane2->GetOriginInWorld(NWU));
    auto chDir2 = internal::Vector3dToChVector(m_plane2->GetNormaleInWorld(NWU));

    auto chrono_item =
        std::dynamic_pointer_cast<chrono::ChLinkMatePlane>(internal::GetChronoConstraint(this));

    chrono_item->Initialize(internal::GetChronoBody2(this),
                            internal::GetChronoBody1(this),
                            false,
                            chPos2,
                            chPos1,
                            chDir2,
                            chDir1);

  }

  void FrConstraintPlaneOnPlane::SetFlipped(bool flip) {
    auto chrono_item =
        std::dynamic_pointer_cast<chrono::ChLinkMatePlane>(internal::GetChronoConstraint(this));
    chrono_item->SetFlipped(flip);
  }

  void FrConstraintPlaneOnPlane::SetDistance(double distance) {
    auto chrono_item =
        std::dynamic_pointer_cast<chrono::ChLinkMatePlane>(internal::GetChronoConstraint(this));
    chrono_item->SetSeparation(distance);
  }

  std::shared_ptr<FrConstraintPlaneOnPlane>
  make_constraint_plane_on_plane(const std::string &name,
                                 FrOffshoreSystem *system,
                                 const std::shared_ptr<FrCPlane> &plane1,
                                 const std::shared_ptr<FrCPlane> &plane2,
                                 bool flipped,
                                 double distance) {

    auto constraint = std::make_shared<FrConstraintPlaneOnPlane>(name, system, plane1, plane2, flipped, distance);
    system->Add(constraint);

    return constraint;
  }

  //------------------------------------------------------------------------------------------------------------------

  FrConstraintPointOnPlane::FrConstraintPointOnPlane(const std::string &name,
                                                     FrOffshoreSystem *system,
                                                     const std::shared_ptr<FrCPlane> &plane,
                                                     const std::shared_ptr<FrCPoint> &point,
                                                     double distance) :
      FrConstraint(name, TypeToString(this), system, plane->GetNode(), point->GetNode()),
      m_plane(plane), m_point(point) {

    m_chronoConstraint = std::make_shared<chrono::ChLinkMateXdistance>();
    SetDistance(distance);
  }

  void FrConstraintPointOnPlane::Initialize() {

    auto chPos2 = internal::Vector3dToChVector(m_point->GetPositionInWorld(NWU));
    auto chPos1 = internal::Vector3dToChVector(m_plane->GetOriginInWorld(NWU));
    auto chDir1 = internal::Vector3dToChVector(m_plane->GetNormaleInWorld(NWU));

    auto chrono_item =
        std::dynamic_pointer_cast<chrono::ChLinkMateXdistance>(internal::GetChronoConstraint(this));

    chrono_item->Initialize(internal::GetChronoBody2(this),
                            internal::GetChronoBody1(this),
                            false,
                            chPos2,
                            chPos1,
                            chDir1);
  }

  void FrConstraintPointOnPlane::SetDistance(double distance) {
    auto chrono_item =
        std::dynamic_pointer_cast<chrono::ChLinkMateXdistance>(internal::GetChronoConstraint(this));
    chrono_item->SetDistance(distance);
  }

  std::shared_ptr<FrConstraintPointOnPlane>
  make_constraint_point_on_plane(const std::string &name,
                                 FrOffshoreSystem *system,
                                 const std::shared_ptr<FrCPlane> &plane,
                                 const std::shared_ptr<FrCPoint> &point,
                                 double distance) {

    auto constraint = std::make_shared<FrConstraintPointOnPlane>(name, system, plane, point, distance);
    system->Add(constraint);

    return constraint;

  }

  //------------------------------------------------------------------------------------------------------------------

  FrConstraintPointOnLine::FrConstraintPointOnLine(const std::string &name,
                                                   FrOffshoreSystem *system,
                                                   const std::shared_ptr<FrCAxis> &line,
                                                   const std::shared_ptr<FrCPoint> &point,
                                                   double distance) :
      FrConstraint(name, TypeToString(this), system, line->GetNode(), point->GetNode()),
      m_point(point), m_axis(line) {

    m_chronoConstraint = std::make_shared<chrono::ChLinkLockPointLine>();
  }

  void FrConstraintPointOnLine::Initialize() {

    auto axisFrame = m_axis->GetNode()->GetFrameInWorld();
    if (m_axis->GetLabel() == YAXIS) axisFrame.RotZ_DEGREES(90, NWU, true);
    if (m_axis->GetLabel() == ZAXIS) axisFrame.RotY_DEGREES(90, NWU, true);


    auto chCoordSys1 = internal::FrFrame2ChCoordsys(axisFrame);
    auto chCoordSys2 = internal::FrFrame2ChCoordsys(m_point->GetNode()->GetFrameInWorld());

    auto chrono_item =
        std::dynamic_pointer_cast<chrono::ChLinkLockPointLine>(internal::GetChronoConstraint(this));

    chrono_item->Initialize(internal::GetChronoBody2(this),
                            internal::GetChronoBody1(this),
                            false,
                            chCoordSys2,
                            chCoordSys1);

  }

  std::shared_ptr<FrConstraintPointOnLine>
  make_constraint_point_on_line(const std::string &name,
                                FrOffshoreSystem *system,
                                const std::shared_ptr<FrCAxis> &line,
                                const std::shared_ptr<FrCPoint> &point) {

    auto constraint = std::make_shared<FrConstraintPointOnLine>(name, system, line, point);
    system->Add(constraint);

    return constraint;

  }


  //------------------------------------------------------------------------------------------------------------------

  FrConstraintDistanceToAxis::FrConstraintDistanceToAxis(const std::string &name,
                                                         FrOffshoreSystem *system,
                                                         const std::shared_ptr<FrCAxis> &axis,
                                                         const std::shared_ptr<FrCPoint> &point,
                                                         bool autoDistance,
                                                         double distance) :
      FrConstraint(name, TypeToString(this), system, axis->GetNode(), point->GetNode()),
      m_point(point), m_axis(axis),
      m_autoDistance(autoDistance) {

    m_chronoConstraint = std::make_shared<chrono::ChLinkRevoluteSpherical>();
    SetDistance(distance);
  }

  void FrConstraintDistanceToAxis::Initialize() {

    auto chPos2 = internal::Vector3dToChVector(m_point->GetPositionInWorld(NWU));
    auto chPos1 = internal::Vector3dToChVector(m_axis->GetOriginInWorld(NWU));
    auto chDir1 = internal::Vector3dToChVector(m_axis->GetDirectionInWorld(NWU));

    auto chrono_item =
        std::dynamic_pointer_cast<chrono::ChLinkRevoluteSpherical>(internal::GetChronoConstraint(this));

    chrono_item->Initialize(internal::GetChronoBody1(this),
                            internal::GetChronoBody2(this),
                            false,
                            chPos1,
                            chDir1,
                            chPos2,
                            m_autoDistance,
                            GetDistance());

  }

  void FrConstraintDistanceToAxis::SetDistance(double distance) {
    m_distance = distance;
  }

  double FrConstraintDistanceToAxis::GetDistance() const {
    return m_distance;
  }

  std::shared_ptr<FrConstraintDistanceToAxis>
  make_constraint_distance_to_axis(const std::string &name,
                                   FrOffshoreSystem *system,
                                   const std::shared_ptr<FrCAxis> &axis,
                                   const std::shared_ptr<FrCPoint> &point,
                                   bool autoDistance,
                                   double distance) {

    auto constraint = std::make_shared<FrConstraintDistanceToAxis>(name, system, axis, point, autoDistance, distance);
    system->Add(constraint);

    return constraint;

  }

  //------------------------------------------------------------------------------------------------------------------

  FrConstraintDistanceBetweenPoints::FrConstraintDistanceBetweenPoints(const std::string &name,
                                                                       FrOffshoreSystem *system,
                                                                       const std::shared_ptr<FrCPoint> &point1,
                                                                       const std::shared_ptr<FrCPoint> &point2,
                                                                       bool autoDistance,
                                                                       double distance) :
      FrConstraint(name, TypeToString(this), system, point1->GetNode(), point2->GetNode()),
      m_point1(point1),
      m_point2(point2),
      m_autoDistance(autoDistance) {

    m_chronoConstraint = std::make_shared<chrono::ChLinkDistance>();
    SetDistance(distance);
  }


  void FrConstraintDistanceBetweenPoints::SetDistance(double distance) {
    m_distance = distance;
  }

  double FrConstraintDistanceBetweenPoints::GetDistance() const {
    return m_distance;
  }

  void FrConstraintDistanceBetweenPoints::Initialize() {

    auto chPos1 = internal::Vector3dToChVector(m_point1->GetPositionInWorld(NWU));
    auto chPos2 = internal::Vector3dToChVector(m_point2->GetPositionInWorld(NWU));

    auto chrono_item =
        std::dynamic_pointer_cast<chrono::ChLinkDistance>(internal::GetChronoConstraint(this));

    chrono_item->Initialize(internal::GetChronoBody1(this),
                            internal::GetChronoBody2(this),
                            false,
                            chPos1,
                            chPos2,
                            m_autoDistance,
                            GetDistance());
  }

  std::shared_ptr<FrConstraintDistanceBetweenPoints>
  make_constraint_distance_between_points(const std::string &name,
                                          FrOffshoreSystem *system,
                                          const std::shared_ptr<FrCPoint> &point1,
                                          const std::shared_ptr<FrCPoint> &point2,
                                          bool autoDistance,
                                          double distance) {

    auto constraint = std::make_shared<FrConstraintDistanceBetweenPoints>(name,
                                                                          system,
                                                                          point1,
                                                                          point2,
                                                                          autoDistance,
                                                                          distance);
    system->Add(constraint);

    return constraint;

  }
}
