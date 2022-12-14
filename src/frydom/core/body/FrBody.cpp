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


#include "FrBody.h"


#include "frydom/logging/FrEventLogger.h"
#include "chrono/assets/ChColorAsset.h"
#include "frydom/core/math/FrMatrix.h"
#include "frydom/core/force/FrForce.h"
#include "frydom/core/common/FrNode.h"
#include "frydom/asset/FrAsset.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/geographicServices/FrGeographicServices.h"
#include "frydom/asset/FrForceAsset.h"
#include "frydom/collision/FrCollisionModel.h"
#include "frydom/logging/FrLogManager.h"
#include "frydom/logging/FrPathManager.h"
#include "frydom/logging/FrTypeNames.h"

#include "frydom/core/common/FrVariablesBodyBase.h"

#include "frydom/core/contact/FrContactInc.h"


namespace frydom {

  namespace internal {

    //
    // FrBodyBase
    //

    FrBodyBase::FrBodyBase(FrBody *body) : chrono::ChBodyAuxRef(), m_frydomBody(body) {}

    FrBodyBase::FrBodyBase(const FrBodyBase &other) : chrono::ChBodyAuxRef(other) {
      m_frydomBody = other.m_frydomBody;
      m_variables_ptr = other.m_variables_ptr;
    }

    void FrBodyBase::SetupInitial() {}

    void FrBodyBase::Update(bool update_assets) {
      chrono::ChBodyAuxRef::Update(update_assets);
      m_frydomBody->Update();
    }

    void FrBodyBase::UpdateAfterMove() {

      auto auxref_to_cog = (chrono::ChFrameMoving<double>) GetFrame_REF_to_COG();

      chrono::ChFrameMoving<double> auxref_to_abs;
      this->TransformLocalToParent(auxref_to_cog, auxref_to_abs);

      SetFrame_REF_to_abs(auxref_to_abs);

      chrono::ChBodyAuxRef::Update(true);

      // Updating markers
      UpdateMarkers(GetChTime());
    }

    void FrBodyBase::UpdateMarkerPositionToCOG(const chrono::ChVector<> newCOG) {

      chrono::ChVector<double> position;

      for (auto &marker : GetMarkerList()) {
        position = marker->GetPos() - newCOG;
        marker->Impose_Rel_Coord(chrono::ChCoordsys<double>(position, marker->GetRot()));
      }
      UpdateMarkers(GetChTime());
    }

    void FrBodyBase::RemoveAsset(std::shared_ptr<chrono::ChAsset> asset) { //taken from RemoveForce
      // trying to remove objects not previously added?

      if (std::find<std::vector<std::shared_ptr<chrono::ChAsset>>::iterator>(assets.begin(), assets.end(), asset) ==
          assets.end()) {
        event_logger::error(m_frydomBody->GetTypeName(), m_frydomBody->GetName(), "Failed to remove asset");
      }

      // warning! linear time search
      assets.erase(
          std::find<std::vector<std::shared_ptr<chrono::ChAsset>>::iterator>(assets.begin(), assets.end(), asset));
    }

    //
    // STATE FUNCTION
    //

    void FrBodyBase::IntToDescriptor(const unsigned int off_v,
                                     const chrono::ChStateDelta &v,
                                     const chrono::ChVectorDynamic<> &R,
                                     const unsigned int off_L,
                                     const chrono::ChVectorDynamic<> &L,
                                     const chrono::ChVectorDynamic<> &Qc) {
      Variables().Get_qb() = v.segment(off_v, 6);
      Variables().Get_fb() = R.segment(off_v, 6);
    }

    void FrBodyBase::IntFromDescriptor(const unsigned int off_v,
                                       chrono::ChStateDelta &v,
                                       const unsigned int off_L,
                                       chrono::ChVectorDynamic<> &L) {
        v.segment(off_v, 6) = Variables().Get_qb();
    }

    //
    // SOLVER FUNCTIONS
    //

    chrono::ChVariables &FrBodyBase::Variables() {

      if (m_variables_ptr) {


        return *m_variables_ptr.get();
      } else {
        return chrono::ChBody::variables;
      }
    }

    chrono::ChVariables *FrBodyBase::GetVariables1() {
      return &Variables();
    }

    void FrBodyBase::SetVariables(const std::shared_ptr<chrono::ChVariables> new_variables) {
      if (variables.IsActive()) {
        m_variables_ptr = new_variables;
        variables.SetDisabled(true);
      }
    }

    void FrBodyBase::InjectVariables(chrono::ChSystemDescriptor &mdescriptor) {
      Variables().SetDisabled(!this->IsActive());
      mdescriptor.InsertVariables(&this->Variables());
    }

    void FrBodyBase::VariablesFbReset() {
      Variables().Get_fb().setZero();
    }

    void FrBodyBase::VariablesFbLoadForces(double factor) {
      // add applied forces to 'fb' vector
      this->Variables().Get_fb().segment(0, 3) += factor * Xforce.eigen();

      // add applied torques to 'fb' vector, including gyroscopic torque
      if (this->GetNoGyroTorque())
        this->Variables().Get_fb().segment(3, 3) += factor * Xtorque.eigen();
      else
        this->Variables().Get_fb().segment(3, 3) += factor * (Xtorque - gyro).eigen();
    }

    void FrBodyBase::VariablesFbIncrementMq() {
      this->Variables().Compute_inc_Mb_v(this->Variables().Get_fb(), this->Variables().Get_qb());
    }

    void FrBodyBase::VariablesQbLoadSpeed() {
      this->Variables().Get_qb().segment(0 ,3) = GetCoord_dt().pos.eigen();
      this->Variables().Get_qb().segment(3, 3) = GetWvel_loc().eigen();
    }

    void FrBodyBase::VariablesQbSetSpeed(double step) {
      chrono::ChCoordsys<> old_coord_dt = this->GetCoord_dt();

      // from 'qb' vector, sets body speed, and updates auxiliary data
      this->SetPos_dt(this->Variables().Get_qb().segment(0, 3));
      this->SetPos_dt(this->Variables().Get_qb().segment(3, 3));

      // apply limits (if in speed clamping mode) to speeds.
      ClampSpeed();

      // compute auxiliary gyroscopic forces
      ComputeGyro();

      // Compute accel. by BDF (approximate by differentiation);
      if (step) {
        this->SetPos_dtdt((this->GetCoord_dt().pos - old_coord_dt.pos) / step);
        this->SetRot_dtdt((this->GetCoord_dt().rot - old_coord_dt.rot) / step);
      }
    }

    void FrBodyBase::VariablesQbIncrementPosition(double dt_step) {
      if (!this->IsActive())
        return;

      // Updates position with incremental action of speed contained in the
      // 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

      chrono::ChVector<> newspeed(Variables().Get_qb().segment(0, 3));
      chrono::ChVector<> newwel(Variables().Get_qb().segment(3, 3));

      // ADVANCE POSITION: pos' = pos + dt * vel
      this->SetPos(this->GetPos() + newspeed * dt_step);

      // ADVANCE ROTATION: rot' = [dt*wwel]%rot  (use quaternion for delta rotation)
      chrono::ChQuaternion<> mdeltarot;
      chrono::ChQuaternion<> moldrot = this->GetRot();
      chrono::ChVector<> newwel_abs = Amatrix * newwel;
      double mangle = newwel_abs.Length() * dt_step;
      newwel_abs.Normalize();
      mdeltarot.Q_from_AngAxis(mangle, newwel_abs);
      chrono::ChQuaternion<> mnewrot = mdeltarot % moldrot;
      this->SetRot(mnewrot);
    }


    std::shared_ptr<frydom::internal::FrBodyBase> GetChronoBody(std::shared_ptr<FrBody> body) {
      return body->m_chronoBody;
    }

    std::shared_ptr<frydom::internal::FrBodyBase> GetChronoBody(FrBody *body) {
      return body->m_chronoBody;
    }

  }  // end namespace frydom::internal

  FrBody::FrBody(const std::string &name, FrOffshoreSystem *system) :
      FrLoggable(name, TypeToString(this), system),
      m_chronoBody(std::make_shared<internal::FrBodyBase>(this)),
      m_DOFMask(std::make_unique<FrDOFMask>(this)) {

    m_chronoBody->SetMaxSpeed(DEFAULT_MAX_SPEED);
    m_chronoBody->SetMaxWvel(DEFAULT_MAX_ROTATION_SPEED);

    SetContactMethod();

    event_logger::info(GetTypeName(), GetName(), "Body created");

  }

  void FrBody::SetFixedInWorld(bool state) {
    m_chronoBody->SetBodyFixed(state);
    if (state) {
      event_logger::info(GetTypeName(), GetName(), "Body set to fixed in world");
    } else {
      event_logger::info(GetTypeName(), GetName(), "Body no more fixed in world");
    }

  }

  bool FrBody::IsFixedInWorld() const {
    return m_chronoBody->GetBodyFixed();
  }

  void FrBody::SetUseSleeping(bool state) {
    m_chronoBody->SetUseSleeping(state);
    if (state) {
      event_logger::info(GetTypeName(), GetName(), "Use sleeping");
    } else {
      event_logger::info(GetTypeName(), GetName(), "Do not use sleeping");
    }
  }

  bool FrBody::GetUseSleeping() const {
    return m_chronoBody->GetUseSleeping();
  }

  void FrBody::SetSleeping(bool state) {
    m_chronoBody->SetSleeping(state);
    if (state) {
      event_logger::info(GetTypeName(), GetName(), "Enter in sleeping mode");
    } else {
      event_logger::info(GetTypeName(), GetName(), "Do not sleep anymore");
    }
  }

  bool FrBody::GetSleeping() const {
    return m_chronoBody->GetSleeping();
  }

  bool FrBody::TrySleeping() {
    return m_chronoBody->TrySleeping();
  }

  bool FrBody::IsActive() {
    return m_chronoBody->IsActive();
  }

  void FrBody::SetupInitial() {
    m_chronoBody->SetupInitial();
    Initialize();
  }

  void FrBody::Initialize() {

    // Check the mass and inertia coefficients
    if (GetInertiaTensor().GetMass() == 0) {
      event_logger::critical(GetTypeName(), GetName(), "Null mass is not permitted");
      exit(EXIT_FAILURE);
    }


    double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    GetInertiaTensor().GetInertiaCoeffsAtCOG(Ixx, Iyy, Izz, Ixy, Ixz, Iyz, NWU);
    if (Ixx == 0 || Iyy == 0. || Izz == 0.) {
      event_logger::critical(GetTypeName(), GetName(), "Null diagonal inertia not permitted");
      exit(EXIT_FAILURE);
    }


    // Initializing forces
    auto forceIter = force_begin();
    for (; forceIter != force_end(); forceIter++) {
      (*forceIter)->Initialize();
    }

    // Initializing nodes
    auto nodeIter = node_begin();
    for (; nodeIter != node_end(); nodeIter++) {
      (*nodeIter)->Initialize();
    }

    // BodyDOF constraints Initialization
    if (m_DOFMask->HasLockedDOF()) {
      InitializeLockedDOF();
    }

    if (m_collisionModel) {
      m_collisionModel->Initialize();
    }

  }

  void FrBody::StepFinalize() {

    // Update the asset
    FrAssetOwner::UpdateAsset();

    // Finalize forces
    auto forceIter = force_begin();
    for (; forceIter != force_end(); forceIter++) {
      (*forceIter)->StepFinalize();
    }

    // Finalize nodes
    auto nodeIter = node_begin();
    for (; nodeIter != node_end(); nodeIter++) {
      (*nodeIter)->StepFinalize();
    }

  }

  void FrBody::Update() {
    // TODO

  }

  // Force linear iterators
  FrBody::ForceIter FrBody::force_begin() {
    return m_externalForces.begin();
  }

  FrBody::ConstForceIter FrBody::force_begin() const {
    return m_externalForces.cbegin();
  }

  FrBody::ForceIter FrBody::force_end() {
    return m_externalForces.end();
  }

  FrBody::ConstForceIter FrBody::force_end() const {
    return m_externalForces.cend();
  }

  // Node linear iterators
  FrBody::NodeIter FrBody::node_begin() {
    return m_nodes.begin();
  }

  FrBody::ConstNodeIter FrBody::node_begin() const {
    return m_nodes.cbegin();
  }

  FrBody::NodeIter FrBody::node_end() {
    return m_nodes.end();
  }

  FrBody::ConstNodeIter FrBody::node_end() const {
    return m_nodes.cend();
  }

  double FrBody::GetMass() const {
    return m_chronoBody->GetMass();
  }

  FrInertiaTensor FrBody::GetInertiaTensor() const {
    double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    SplitMatrix33IntoCoeffs(m_chronoBody->GetInertia(),
                            Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz);

    return {GetMass(), Ixx, Iyy, Izz, Ixy, Ixz, Iyz, GetCOG(NWU), NWU};
  }

  void FrBody::SetInertiaTensor(const FrInertiaTensor &inertia) {

    m_chronoBody->SetMass(inertia.GetMass());

    auto cog_pos = inertia.GetCOGPosition(NWU);

    SetCOG(cog_pos, NWU);

    double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    inertia.GetInertiaCoeffsAtCOG(Ixx, Iyy, Izz, Ixy, Ixz, Iyz, NWU);

    m_chronoBody->SetInertiaXX(chrono::ChVector<double>(Ixx, Iyy, Izz));
    m_chronoBody->SetInertiaXY(chrono::ChVector<double>(Ixy, Ixz, Iyz));

    event_logger::info(GetTypeName(), GetName(),
                       "Set inertia tensor with mass = {} kg, G = [{}\t{}\t{}], "
                       "Ixx = {}, Iyy = {}, Izz = {}, Ixy = {}, Ixz = {}, Iyz = {}",
                       inertia.GetMass(),
                       cog_pos[0], cog_pos[1], cog_pos[2],
                       Ixx, Iyy, Izz, Ixy, Ixz, Iyz);

  }

  void FrBody::AllowCollision(bool isColliding) {
    m_chronoBody->SetCollide(isColliding);
    if (isColliding) {
      event_logger::info(GetTypeName(), GetName(), "Collision activated");
    } else {
      m_collisionModel.reset();
      event_logger::info(GetTypeName(), GetName(), "Collision deactivated");
    }
  }

  FrCollisionModel *FrBody::GetCollisionModel() {
    return m_collisionModel.get();
  }

  void FrBody::SetCollisionModel(std::shared_ptr<FrCollisionModel> collisionModel) {
    m_chronoBody->SetCollisionModel(collisionModel->m_chronoCollisionModel);
    AllowCollision(true);
  }

  Force FrBody::GetContactForceInWorld(FRAME_CONVENTION fc) {
    auto force = GetSystem()->GetContactForceOnBodyInWorld(this, fc);
    return force;
  }

  void FrBody::ActivateSpeedLimits(bool activate) {
    m_chronoBody->SetLimitSpeed(activate);
    if (activate) {
      event_logger::info(GetTypeName(), GetName(), "Speed limit activated");
    } else {
      event_logger::info(GetTypeName(), GetName(), "Speed limit deactivated");
    }
  }

  void FrBody::SetMaxSpeed(double maxSpeed_ms) {
    m_chronoBody->SetMaxSpeed((float) maxSpeed_ms);
    ActivateSpeedLimits(true);
    event_logger::info(GetTypeName(), GetName(), "Speed limit set to {} m/s", maxSpeed_ms);
  }

  void FrBody::SetMaxRotationSpeed(double wMax_rads) {
    m_chronoBody->SetMaxWvel((float) wMax_rads);
    ActivateSpeedLimits(true);
    event_logger::info(GetTypeName(), GetName(), "Speed limit set to {} rad/s", wMax_rads);
  }

  void FrBody::RemoveGravity(
      bool val) { // TODO : ajouter la force d'accumulation a l'initialisation --> cas ou le systeme n'a pas encore ete precise pour la gravite...
    // TODO : this method should not be used in production !!
    if (val) {
      m_chronoBody->Accumulate_force(
          GetMass() * m_chronoBody->TransformDirectionParentToLocal(chrono::ChVector<double>(0., 0., 9.81)),
          chrono::VNULL,
          true
      );
      // TODO : aller chercher la gravite dans systeme !!!
    } else {
      m_chronoBody->Empty_forces_accumulators();
    }

  }

  void FrBody::AddExternalForce(std::shared_ptr<frydom::FrForce> force) {
    /// This subroutine is used for adding the hydrodynamic loads.
    m_chronoBody->AddForce(internal::GetChronoForce(force));  // FrBody is a friend class of FrForce
    m_externalForces.push_back(force);
    GetSystem()->GetPathManager()->RegisterTreeNode(force.get());

    GetSystem()->GetLogManager()->Add(force);

    event_logger::info(GetTypeName(), GetName(), "External force {} added", force->GetName());

  }

  void FrBody::RemoveExternalForce(std::shared_ptr<FrForce> force) {
    m_chronoBody->RemoveForce(internal::GetChronoForce(force));

    m_externalForces.erase(
        std::find<std::vector<std::shared_ptr<FrForce>>::iterator>(m_externalForces.begin(), m_externalForces.end(),
                                                                   force));

    event_logger::info(GetTypeName(), GetName(), "External force {} removed", force->GetName());

    auto asset = force->GetAsset();

    if (asset) {
      m_chronoBody->RemoveAsset(internal::GetChronoAsset(asset));

      bool asserted = false;
      for (int ia = 0; ia < m_assets.size(); ++ia) {
        if (m_assets[ia] == asset) {
          m_assets.erase(m_assets.begin() + ia);
          asserted = true;
        }
      }
      assert(asserted); // FIXME : renvoyer une erreur mais ne pas faire planter !!!
      asset = nullptr;

    }

    auto logManager = GetSystem()->GetLogManager();
    if (auto loggable = std::dynamic_pointer_cast<FrLoggableBase>(force)) {
      logManager->Remove(loggable);
    }

  }

  void FrBody::RemoveAllForces() {
    m_chronoBody->RemoveAllForces();

    auto logManager = GetSystem()->GetLogManager();
    auto pathManager = GetSystem()->GetPathManager();
    for (auto &force : m_externalForces) {
      logManager->Remove(force);
    }
    m_externalForces.clear();

    event_logger::info(GetTypeName(), GetName(), "All forces removed");
  }

  void FrBody::RemoveNode(std::shared_ptr<FrNode> node) {
    m_chronoBody->RemoveMarker(internal::GetChronoMarker(node));

    GetSystem()->GetLogManager()->Remove(node);
    GetSystem()->GetPathManager()->UnregisterTreeNode(node.get());

    event_logger::info(GetTypeName(), GetName(), "Node {} has been removed", node->GetName());

  }

  void FrBody::RemoveAllNodes() {
    for (auto &node : m_nodes) {
      RemoveNode(node);
    }
    m_nodes.clear();

    event_logger::info(GetTypeName(), GetName(), "All nodes removed");
  }

  Force FrBody::GetTotalExtForceInWorld(FRAME_CONVENTION fc) const {
    Force force = m_chronoBody->GetAppliedForce().eigen();
    if (IsNED(fc)) return internal::SwapFrameConvention<Position>(force);
    return force;
  }

  Force FrBody::GetTotalExtForceInBody(FRAME_CONVENTION fc) const {
    return ProjectVectorInBody(GetTotalExtForceInWorld(fc), fc);
  }

  Torque FrBody::GetTotalExtTorqueInBodyAtCOG(FRAME_CONVENTION fc) const {
    auto torque = m_chronoBody->GetAppliedTorque().eigen();
    if (IsNED(fc)) return internal::SwapFrameConvention<Position>(torque);
    return torque;
  }

  Torque FrBody::GetTotalExtTorqueInWorldAtCOG(FRAME_CONVENTION fc) const {
    return ProjectVectorInWorld(GetTotalExtTorqueInBodyAtCOG(fc), fc);
  }


  // Nodes

  std::shared_ptr<FrNode> FrBody::NewNode(const std::string &name) {
    auto node = std::make_shared<FrNode>(name, this);
    m_nodes.push_back(node);

//    SetPosition(GetPosition(NWU), NWU);
    node->SetPositionInBody({0., 0., 0.}, NWU);

    auto pos = node->GetNodePositionInBody(NWU);

    event_logger::info(GetTypeName(), GetName(),
                       "Node {} added to body at position {}\t{}\t{}",
                       node->GetName(), pos[0], pos[1], pos[2]);
    GetSystem()->GetPathManager()->RegisterTreeNode(node.get());

    GetSystem()->GetLogManager()->Add(node);

    return node;
  }

  void FrBody::SetCOG(const Position &bodyPos, FRAME_CONVENTION fc) {
    FrFrame cogFrame;
    cogFrame.SetPosition(bodyPos, fc);
    event_logger::info(GetTypeName(), GetName(),
                       "Center of gravity set (in body reference frame) to [{}\t{}\t{}]", bodyPos.x(), bodyPos.y(),
                       bodyPos.z());
//    m_chronoBody->UpdateMarkerPositionToCOG(internal::Vector3dToChVector(cogFrame.GetPosition(NWU)));
    m_chronoBody->SetFrame_COG_to_REF(internal::FrFrame2ChFrame(cogFrame));
  }

  Position FrBody::GetCOG(FRAME_CONVENTION fc) const {
    Position cogPos = m_chronoBody->GetFrame_COG_to_REF().GetPos().eigen(); // In NWU
    if (IsNED(fc)) return internal::SwapFrameConvention<Position>(cogPos);
    return cogPos;
  }

  Position FrBody::GetPosition(FRAME_CONVENTION fc) const {
    Position refPos = m_chronoBody->GetFrame_REF_to_abs().GetPos().eigen();
    if (IsNED(fc)) return internal::SwapFrameConvention<Position>(refPos);
    return refPos;
  }

  FrGeographicCoord FrBody::GetGeoPosition(FRAME_CONVENTION fc) const {
    return CartToGeo(GetPosition(fc), fc);
  }

  void FrBody::SetPosition(const Position &worldPos, FRAME_CONVENTION fc) {

    /// This subroutine sets the initial position of a body in world.

    auto bodyFrame = GetFrame();
    bodyFrame.SetPosition(worldPos, fc);

    event_logger::info(GetTypeName(), GetName(),
                       "Set body position (in world reference frame NWU) to [{}\t{}\t{}]",
                       worldPos.x(), worldPos.y(), worldPos.z());

    m_chronoBody->SetFrame_REF_to_abs(internal::FrFrame2ChFrame(bodyFrame));
    m_chronoBody->UpdateAfterMove();
  }

  void FrBody::SetPosition(const Position &worlRefPosition,
                           const double &heading,
                           const double &distance,
                           ANGLE_UNIT unit,
                           FRAME_CONVENTION fc) {
    double alpha = heading;
    if (unit == DEG) alpha *= DEG2RAD;

    Direction direction = {std::cos(alpha), std::sin(alpha), 0.};
    SetPosition(worlRefPosition + direction * distance, fc);
  }

  void FrBody::SetGeoPosition(const FrGeographicCoord &geoCoord) {
    SetPosition(GeoToCart(geoCoord, NWU), NWU);
    event_logger::info(GetTypeName(), GetName(),
                       "Set geographic body position to latitude = {} ??; longitude{} ??]",
                       geoCoord.GetLatitude(), geoCoord.GetLongitude());
  }

  FrRotation FrBody::GetRotation() const {
    return FrRotation(GetQuaternion());
  }

  void FrBody::SetRotation(const FrRotation &rotation) {
    SetRotation(rotation.GetQuaternion());
  }

  FrUnitQuaternion FrBody::GetQuaternion() const {
    return internal::Ch2FrQuaternion(m_chronoBody->GetRot());
  }

  void FrBody::SetRotation(const FrUnitQuaternion &quaternion) {
    Position bodyWorldPos = GetPosition(NWU);
    m_chronoBody->SetRot(internal::Fr2ChQuaternion(quaternion));

    double phi, theta, psi;
    FrRotation(quaternion).GetCardanAngles_DEGREES(phi, theta, psi, NWU);

    event_logger::info(GetTypeName(), GetName(),
                       "Set body orientation (in world reference frame NWU, cardan angles) to "
                       "phi = {}\ttheta = {}\tpsi = {}]",
                       phi, theta, psi);

    SetPosition(bodyWorldPos, NWU);  // FIXME : pourquoi doit-on set la position ???
  }

  FrFrame FrBody::GetFrame() const {
    FrFrame bodyRefFrame;
    bodyRefFrame.SetPosition(GetPosition(NWU), NWU);
    bodyRefFrame.SetRotation(GetQuaternion());
    return bodyRefFrame;
  }

  void FrBody::SetFrame(const FrFrame &worldFrame) {
    SetPosition(worldFrame.GetPosition(NWU), NWU);
    SetRotation(worldFrame.GetQuaternion());
  }

  FrFrame FrBody::GetFrameAtPoint(const Position &bodyPoint, FRAME_CONVENTION fc) const {
    FrFrame pointFrame;
    pointFrame.SetPosition(GetPointPositionInWorld(bodyPoint, fc), fc);
    pointFrame.SetRotation(GetQuaternion());
    return pointFrame;
  }

  FrFrame FrBody::GetFrameAtCOG() const {
    return GetFrameAtPoint(GetCOG(NWU), NWU);
  }

  FrFrame FrBody::GetHeadingFrame() const {
    FrFrame headingFrame;
    double phi, theta, psi;
    GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, NWU);
    headingFrame.SetRotZ_RADIANS(psi, NWU);
    headingFrame.SetPosition(GetCOGPositionInWorld(NWU), NWU);
    return headingFrame;
  }

  Position FrBody::GetPointPositionInWorld(const Position &bodyPos, FRAME_CONVENTION fc) const {
    return GetPosition(fc) + ProjectVectorInWorld<Position>(bodyPos, fc);
  }

  Position FrBody::GetPointPositionInBody(const Position &worldPos, FRAME_CONVENTION fc) const {
    return ProjectVectorInBody<Position>(worldPos - GetPosition(fc), fc);
  }

  Position FrBody::GetCOGPositionInWorld(FRAME_CONVENTION fc) const {
    Position cogPos = internal::ChVectorToVector3d<Position>(m_chronoBody->GetPos());
    if (IsNED(fc)) return internal::SwapFrameConvention<Position>(cogPos);
    return cogPos;
  }

  FrGeographicCoord FrBody::GetGeoPointPositionInWorld(const Position &bodyPos, FRAME_CONVENTION fc) const {
    return CartToGeo(GetPointPositionInWorld(bodyPos, fc), fc);
  }

  FrGeographicCoord FrBody::GetGeoPointPositionInBody(const Position &worldPos, FRAME_CONVENTION fc) const {
    return CartToGeo(GetPointPositionInBody(worldPos, fc), fc);
  }

  FrGeographicCoord FrBody::GetCOGGeoPosition() const {
    return CartToGeo(GetCOGPositionInWorld(NWU), NWU);
  }


  void FrBody::SetPositionOfBodyPoint(const Position &bodyPoint, const Position &worldPos, FRAME_CONVENTION fc) {
    Position bodyWorldPos = GetPosition(fc);
    Position worldPointPos = GetPointPositionInWorld(bodyPoint, fc);

    Translation translation = worldPos - worldPointPos;
    TranslateInWorld(translation, fc);
  }

  void FrBody::TranslateInWorld(const Translation &worldTranslation, FRAME_CONVENTION fc) {
    auto refFrame = GetFrame();
    refFrame.SetPosition(refFrame.GetPosition(fc) + worldTranslation, fc);
    m_chronoBody->SetFrame_REF_to_abs(internal::FrFrame2ChFrame(refFrame));
    m_chronoBody->UpdateAfterMove();
  }

  void FrBody::TranslateInWorld(double x, double y, double z, FRAME_CONVENTION fc) {
    TranslateInWorld(Translation(x, y, z), fc);
  }

  void FrBody::TranslateInBody(const Translation &bodyTranslation, FRAME_CONVENTION fc) {
    auto refFrame = GetFrame();
    refFrame.SetPosition(refFrame.GetPosition(fc) + ProjectVectorInWorld<Position>(bodyTranslation, fc), fc);
    m_chronoBody->SetFrame_REF_to_abs(internal::FrFrame2ChFrame(refFrame));
    m_chronoBody->UpdateAfterMove();
  }

  void FrBody::TranslateInBody(double x, double y, double z, FRAME_CONVENTION fc) {
    TranslateInBody(Translation(x, y, z), fc);
  }

  void FrBody::Rotate(const FrRotation &relRotation) {
    SetRotation(GetRotation() * relRotation);
  }

  void FrBody::Rotate(const FrUnitQuaternion &relQuaternion) {
    SetRotation(GetQuaternion() * relQuaternion);
  }

  void FrBody::RotateAroundPointInWorld(const FrRotation &rot, const Position &worldPos, FRAME_CONVENTION fc) {
    RotateAroundPointInWorld(rot.GetQuaternion(), worldPos, fc);
  }

  void FrBody::RotateAroundPointInBody(const FrRotation &rot, const Position &bodyPos, FRAME_CONVENTION fc) {
    RotateAroundPointInBody(rot.GetQuaternion(), bodyPos, fc);
  }

  void FrBody::RotateAroundPointInWorld(const FrUnitQuaternion &rot, const Position &worldPos, FRAME_CONVENTION fc) {
    Position bodyPos = GetPointPositionInBody(worldPos, fc);
    Rotate(rot);
    SetPositionOfBodyPoint(bodyPos, worldPos, fc);
  }

  void FrBody::RotateAroundPointInBody(const FrUnitQuaternion &rot, const Position &bodyPos, FRAME_CONVENTION fc) {
    Position worldPos = GetPointPositionInWorld(bodyPos, fc);
    Rotate(rot);
    SetPositionOfBodyPoint(bodyPos, worldPos, fc);
  }

  void FrBody::RotateAroundCOG(const FrRotation &rot, FRAME_CONVENTION fc) {
    RotateAroundPointInBody(rot, GetCOG(fc), fc);
  }

  void FrBody::RotateAroundCOG(const FrUnitQuaternion &rot, FRAME_CONVENTION fc) {
    RotateAroundPointInBody(rot, GetCOG(fc), fc);
  }

  void FrBody::SetGeneralizedVelocityInWorld(const Velocity &worldVel, const AngularVelocity &worldAngVel,
                                             FRAME_CONVENTION fc) {
    SetGeneralizedVelocityInWorldAtPointInBody(Position(0., 0., 0.), worldVel, worldAngVel, fc);
  }

  void FrBody::SetGeneralizedVelocityInBody(const Velocity &bodyVel, const AngularVelocity &bodyAngVel,
                                            FRAME_CONVENTION fc) {
    SetGeneralizedVelocityInBodyAtPointInBody(Position(0., 0., 0.), bodyVel, bodyAngVel, fc);
  }

  Velocity FrBody::GetLinearVelocityInWorld(FRAME_CONVENTION fc) const {
    Velocity bodyVel = internal::ChVectorToVector3d<Velocity>(m_chronoBody->GetFrame_REF_to_abs().GetPos_dt());
    if (IsNED(fc)) return internal::SwapFrameConvention<Velocity>(bodyVel);
    return bodyVel;
  }

  Velocity FrBody::GetVelocityInBody(FRAME_CONVENTION fc) const {
    return ProjectVectorInBody<Velocity>(GetLinearVelocityInWorld(fc), fc);
  }

  void FrBody::SetVelocityInWorldNoRotation(const Velocity &worldVel, FRAME_CONVENTION fc) {
    auto worldVelTmp = worldVel;
    if (IsNED(fc)) worldVelTmp = internal::SwapFrameConvention<Velocity>(worldVelTmp);
    chrono::ChCoordsys<double> coord;
    coord.pos = internal::Vector3dToChVector(worldVelTmp);
    coord.rot.SetNull();
    m_chronoBody->SetCoord_dt(coord);
    m_chronoBody->UpdateAfterMove();
  }

  void FrBody::SetVelocityInBodyNoRotation(const Velocity &bodyVel, FRAME_CONVENTION fc) {
    SetVelocityInWorldNoRotation(ProjectVectorInWorld(bodyVel, fc), fc);
  }

  Velocity FrBody::GetCOGLinearVelocityInWorld(FRAME_CONVENTION fc) const {
    Velocity cogVel = internal::ChVectorToVector3d<Velocity>(m_chronoBody->GetCoord_dt().pos); // In NWU
    if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(cogVel);
    return cogVel;
  }

  Velocity FrBody::GetCOGVelocityInBody(FRAME_CONVENTION fc) const {
    return ProjectVectorInBody<Velocity>(GetCOGLinearVelocityInWorld(fc), fc);
  }

  Velocity FrBody::GetVelocityInHeadingFrame(FRAME_CONVENTION fc) const {
    return GetHeadingFrame().ProjectVectorParentInFrame(GetCOGLinearVelocityInWorld(fc), fc);
  }

  void FrBody::SetAccelerationInWorldNoRotation(const Acceleration &worldAcc, FRAME_CONVENTION fc) {
    auto worldAccTmp = worldAcc;
    if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(worldAccTmp);
    chrono::ChCoordsys<double> coord;
    coord.pos = internal::Vector3dToChVector(worldAccTmp);
    coord.rot.SetNull();
    m_chronoBody->SetCoord_dtdt(coord);
    m_chronoBody->UpdateAfterMove();
  }

  void FrBody::SetAccelerationInBodyNoRotation(const Acceleration &bodyAcc, FRAME_CONVENTION fc) {
    SetAccelerationInWorldNoRotation(ProjectVectorInWorld<Acceleration>(bodyAcc, fc), fc);
  }

  Acceleration FrBody::GetLinearAccelerationInWorld(FRAME_CONVENTION fc) const {
    return GetAccelerationInWorldAtPointInBody(Position(0., 0., 0.), fc);
  }

  Acceleration FrBody::GetAccelerationInBody(FRAME_CONVENTION fc) const {
    return ProjectVectorInBody<Acceleration>(GetLinearAccelerationInWorld(fc), fc);
  }


  Acceleration FrBody::GetCOGLinearAccelerationInWorld(FRAME_CONVENTION fc) const {
    Acceleration cogAcc = internal::ChVectorToVector3d<Acceleration>(m_chronoBody->GetCoord_dtdt().pos); // In NWU
    if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(cogAcc);
    return cogAcc;
  }

  Acceleration FrBody::GetCOGAccelerationInBody(FRAME_CONVENTION fc) const {
    return ProjectVectorInBody<Acceleration>(GetCOGLinearAccelerationInWorld(fc), fc);
  }

  void FrBody::SetAngularVelocityInWorld(const AngularVelocity &worldAngVel, FRAME_CONVENTION fc) {
    auto worldAngVelTmp = worldAngVel;
    if (IsNED(fc)) internal::SwapFrameConvention<AngularVelocity>(worldAngVelTmp);
    m_chronoBody->SetWvel_par(internal::Vector3dToChVector(worldAngVelTmp));
    m_chronoBody->UpdateAfterMove();
  }

  void FrBody::SetCOGAngularVelocityInWorld(const AngularVelocity &worldAngVel, FRAME_CONVENTION fc) {
    auto worldAngVelTmp = worldAngVel;
    if (IsNED(fc)) internal::SwapFrameConvention<AngularVelocity>(worldAngVelTmp);
    m_chronoBody->SetWvel_par(internal::Vector3dToChVector(worldAngVelTmp));
  }

  void FrBody::SetAngularVelocityInBody(const AngularVelocity &bodyAngVel, FRAME_CONVENTION fc) {
    SetAngularVelocityInWorld(ProjectVectorInWorld(bodyAngVel, fc), fc);
  }

  AngularVelocity FrBody::GetAngularVelocityInWorld(FRAME_CONVENTION fc) const {
    AngularVelocity angVel = internal::ChVectorToVector3d<AngularVelocity>(m_chronoBody->GetWvel_par());
    if (IsNED(fc)) internal::SwapFrameConvention<AngularVelocity>(angVel);
    return angVel;
  }

  AngularVelocity FrBody::GetAngularVelocityInBody(FRAME_CONVENTION fc) const {
    return ProjectVectorInBody<AngularVelocity>(GetAngularVelocityInWorld(fc), fc);
  }

  void FrBody::SetAngularAccelerationInWorld(const AngularAcceleration &worldAngAcc, FRAME_CONVENTION fc) {
    auto worldAngAccTmp = worldAngAcc;
    if (IsNED(fc)) internal::SwapFrameConvention<AngularAcceleration>(worldAngAccTmp);
    auto chronoAngAcc = internal::Vector3dToChVector(worldAngAccTmp);
    m_chronoBody->SetWacc_par(
        chronoAngAcc); // FIXME : dans chrono, l'argument d'entree n'est pas const... -> fix Chrono
    m_chronoBody->UpdateAfterMove();
  }

  void FrBody::SetAngularAccelerationInBody(const AngularAcceleration &bodyAngAcc, FRAME_CONVENTION fc) {
    SetAngularAccelerationInWorld(ProjectVectorInWorld(bodyAngAcc, fc), fc);
  }

  AngularAcceleration FrBody::GetAngularAccelerationInWorld(FRAME_CONVENTION fc) const {
    AngularAcceleration angAcc = internal::ChVectorToVector3d<AngularAcceleration>(m_chronoBody->GetWacc_par());
    if (IsNED(fc)) internal::SwapFrameConvention<AngularAcceleration>(angAcc);
    return angAcc;
  }

  AngularAcceleration FrBody::GetAngularAccelerationInBody(FRAME_CONVENTION fc) const {
    return ProjectVectorInBody(GetAngularAccelerationInWorld(fc), fc);
  }

  Velocity FrBody::GetVelocityInWorldAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {
    Position bodyPoint = GetPointPositionInBody(worldPoint, fc);
    return GetVelocityInWorldAtPointInBody(bodyPoint, fc);
  }

  Velocity FrBody::GetVelocityInWorldAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {
    return ProjectVectorInWorld<Velocity>(GetVelocityInBodyAtPointInBody(bodyPoint, fc), fc);
  }

  Velocity FrBody::GetVelocityInBodyAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {
    Position bodyPoint = GetPointPositionInBody(worldPoint, fc);
    return GetVelocityInBodyAtPointInBody(bodyPoint, fc);
  }

  Velocity FrBody::GetVelocityInBodyAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {
    Velocity bodyVel = GetVelocityInBody(fc);
    AngularVelocity bodyAngVel = GetAngularVelocityInBody(fc);
    return bodyVel + bodyAngVel.cross(bodyPoint);
  }

  Acceleration FrBody::GetAccelerationInWorldAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {
    auto bodyPoint = GetPointPositionInBody(worldPoint, fc);
    return GetAccelerationInWorldAtPointInBody(bodyPoint, fc);
  }

  Acceleration FrBody::GetAccelerationInWorldAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {
    auto bodyPointTmp = bodyPoint;
    if (IsNED(fc)) internal::SwapFrameConvention<Position>(bodyPointTmp);

    Acceleration pointAcc = internal::ChVectorToVector3d<Acceleration>(
        m_chronoBody->PointAccelerationLocalToParent(internal::Vector3dToChVector(bodyPointTmp - GetCOG(NWU)))
    );

    if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(pointAcc);
    return pointAcc;
  }

  Acceleration FrBody::GetAccelerationInBodyAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {
    return GetAccelerationInBodyAtPointInBody(GetPointPositionInBody(worldPoint, fc), fc);
  }

  Acceleration FrBody::GetAccelerationInBodyAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {
    return ProjectVectorInBody(GetAccelerationInWorldAtPointInBody(bodyPoint, fc), fc);
  }


  void FrBody::SetGeneralizedVelocityInWorldAtPointInWorld(const Position &worldPoint, const Velocity &worldVel,
                                                           const AngularVelocity &worldAngVel, FRAME_CONVENTION fc) {
    Position bodyPoint = GetPointPositionInBody(worldPoint, fc);
    SetGeneralizedVelocityInWorldAtPointInBody(bodyPoint, worldVel, worldAngVel, fc);
  }

  void FrBody::SetGeneralizedVelocityInWorldAtPointInBody(const Position &bodyPoint, const Velocity &worldVel,
                                                          const AngularVelocity &worldAngVel, FRAME_CONVENTION fc) {
    Velocity bodyVel = ProjectVectorInBody<Velocity>(worldVel, fc);
    AngularVelocity bodyAngVel = ProjectVectorInBody<AngularVelocity>(worldAngVel, fc);
    SetGeneralizedVelocityInBodyAtPointInBody(bodyPoint, bodyVel, bodyAngVel, fc);
  }

  void FrBody::SetGeneralizedVelocityInBodyAtPointInWorld(const Position &worldPoint, const Velocity &bodyVel,
                                                          const AngularVelocity &bodyAngVel, FRAME_CONVENTION fc) {
    Position bodyPoint = GetPointPositionInBody(worldPoint, fc);
    SetGeneralizedVelocityInBodyAtPointInBody(bodyPoint, bodyVel, bodyAngVel, fc);
  }

  void FrBody::SetGeneralizedVelocityInBodyAtPointInBody(const Position &bodyPoint, const Velocity &bodyVel,
                                                         const AngularVelocity &bodyAngVel, FRAME_CONVENTION fc) {

    Position PG = GetCOG(fc) - bodyPoint;
    Velocity cogVel = bodyVel + bodyAngVel.cross(PG);
    SetVelocityInBodyNoRotation(cogVel, fc);
    SetAngularVelocityInBody(bodyAngVel, fc);
  }

  void
  FrBody::SetGeneralizedAccelerationInBodyAtCOG(const Acceleration &bodyAcc, const AngularAcceleration &bodyAngAcc,
                                                FRAME_CONVENTION fc) {
    SetAccelerationInBodyNoRotation(bodyAcc, fc);
    SetAngularAccelerationInBody(bodyAngAcc, fc);
  }

  void
  FrBody::SetGeneralizedAccelerationInWorldAtCOG(const Acceleration &worldAcc, const AngularAcceleration &worldAngAcc,
                                                 FRAME_CONVENTION fc) {
    SetAccelerationInWorldNoRotation(worldAcc, fc);
    SetAngularAccelerationInWorld(worldAngAcc, fc);
  }

  void FrBody::CartToGeo(const Position &cartPos, FrGeographicCoord &geoCoord, FRAME_CONVENTION fc) const {
    geoCoord = CartToGeo(cartPos, fc);
  }

  void FrBody::CartToGeo(const Position &cartPos, FrGeographicCoord &geoCoord, FRAME_CONVENTION fc) {
    geoCoord = CartToGeo(cartPos, fc);
  }

  FrGeographicCoord FrBody::CartToGeo(const Position &cartPos, FRAME_CONVENTION fc) const {
    return GetSystem()->GetEnvironment()->GetGeographicServices()->CartToGeo(cartPos, fc);
  }

  FrGeographicCoord FrBody::CartToGeo(const Position &cartPos, FRAME_CONVENTION fc) {
    return GetSystem()->GetEnvironment()->GetGeographicServices()->CartToGeo(cartPos, fc);
  }


  void FrBody::GeoToCart(const FrGeographicCoord &geoCoord, Position &cartPos, FRAME_CONVENTION fc) const {
    cartPos = GeoToCart(geoCoord, fc);
  }

  void FrBody::GeoToCart(const FrGeographicCoord &geoCoord, Position &cartPos, FRAME_CONVENTION fc) {
    cartPos = GeoToCart(geoCoord, fc);
  }

  Position FrBody::GeoToCart(const FrGeographicCoord &geoCoord, FRAME_CONVENTION fc) const {
    return GetSystem()->GetEnvironment()->GetGeographicServices()->GeoToCart(geoCoord, fc);
  }

  Position FrBody::GeoToCart(const FrGeographicCoord &geoCoord, FRAME_CONVENTION fc) {
    return GetSystem()->GetEnvironment()->GetGeographicServices()->GeoToCart(geoCoord, fc);
  }

  void FrBody::SetContactMethod() {
    switch (GetSystem()->GetSystemType()) {
      case FrOffshoreSystem::SYSTEM_TYPE::SMOOTH_CONTACT:
        m_chronoBody->GetCollisionModel()->SetAllShapesMaterial(std::make_shared<chrono::ChMaterialSurfaceSMC>());
        event_logger::info(GetTypeName(), GetName(), "Contact method set to SMOOTH");
        break;
      case FrOffshoreSystem::SYSTEM_TYPE::NONSMOOTH_CONTACT:
        m_chronoBody->GetCollisionModel()->SetAllShapesMaterial(std::make_shared<chrono::ChMaterialSurfaceNSC>());
        event_logger::info(GetTypeName(), GetName(), "Contact method set to NON SMOOTH");
        break;
    };


  }

  void FrBody::InitializeLockedDOF() {

    // TODO : voir si n'accepte pas de definir des offset sur les ddl bloques...

    if (m_DOFLink) {
      // updating the link with the new mask
      m_DOFLink->SetDOFMask(m_DOFMask.get());
      m_DOFLink->Initialize();
    }
    else {

      // Getting the markers that enter in the link

      // Body marker placed at the current COG body position  // TODO : voir si on se donne d'autres regles que le COG...
      auto bodyNode = NewNode(GetName() + "_locking_node");

      auto cogPositionInWorld = GetCOGPositionInWorld(NWU);
      auto bodyOrientationInWorld = GetQuaternion();

      auto bodyNodeFrameInWorld = FrFrame(cogPositionInWorld, bodyOrientationInWorld, NWU);
      bodyNode->SetFrameInWorld(bodyNodeFrameInWorld);

      // World Marker placed at the current COG body position
      auto node_numbers = GetSystem()->GetWorldBody()->GetNodeList().size();
      auto worldNode = GetSystem()->GetWorldBody()->NewNode("world_body_locking_node" + std::to_string(node_numbers));
      worldNode->SetFrameInBody(bodyNodeFrameInWorld);

      // Creating the link
      m_DOFLink = std::make_shared<FrDOFMaskLink>(GetName() + "_locking_constraint", GetSystem(), bodyNode, worldNode);

      // Initializing the link with the DOFMask
      m_DOFLink->SetDOFMask(m_DOFMask.get());

      bodyNode->LogThis(m_DOFMask->IsLogged());
      worldNode->LogThis(m_DOFMask->IsLogged());
      m_DOFLink->LogThis(m_DOFMask->IsLogged());

      // Adding the link to the system
      GetSystem()->Add(m_DOFLink);
    }
  }

  FrDOFMask *FrBody::GetDOFMask() {
    return m_DOFMask.get();
  }

  void FrBody::DefineLogMessages() {

    // TODO : changer le nom du message. Le message protobuf associe doit avoir le nom frydom_msg.FrBody
    auto msg = NewMessage("FrBody", "Body states");

    msg->AddField<double>("Time", "s", "Current time of the simulation", [this]() { return GetSystem()->GetTime(); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("PositionInWorld", "m", fmt::format("body position in the world reference frame in {}", GetLogFC()),
         [this]() { return GetPosition(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("COGPositionInWorld", "m",
         fmt::format("COG body position in the world reference frame in {}", GetLogFC()),
         [this]() { return GetCOGPositionInWorld(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("CardanAngles", "deg",
         fmt::format("body orientation in the world reference frame in {}", GetLogFC()),
         [this]() {
           double phi, theta, psi;
           GetRotation().GetCardanAngles_DEGREES(phi, theta, psi, GetLogFC());
           phi = mathutils::Normalize__180_180(phi);
           theta = mathutils::Normalize__180_180(theta);
           psi = mathutils::Normalize__180_180(psi);
           return Vector3d<double>(phi, theta, psi);
         });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("VelocityInBody", "m/s",
         fmt::format("body linear velocity in the body local frame in {}", GetLogFC()),
         [this]() { return GetVelocityInBody(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("COGVelocityInBody", "m/s",
         fmt::format("COG linear velocity in the body local frame in {}", GetLogFC()),
         [this]() { return GetCOGVelocityInBody(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("LinearVelocityInWorld", "m/s",
         fmt::format("body linear velocity in the world reference frame in {}", GetLogFC()),
         [this]() { return GetLinearVelocityInWorld(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("COGLinearVelocityInWorld", "m/s",
         fmt::format("COG body linear velocity in the world reference frame in {}", GetLogFC()),
         [this]() { return GetCOGLinearVelocityInWorld(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("AngularVelocityInWorld", "rad/s",
         fmt::format("body angular velocity in the world reference frame in {}", GetLogFC()),
         [this]() { return GetAngularVelocityInWorld(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("LinearAccelerationInWorld", "m/s2",
         fmt::format("body linear acceleration in the world reference frame in {}", GetLogFC()),
         [this]() { return GetLinearAccelerationInWorld(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("LinearAccelerationInBody", "m/s2",
         fmt::format("body linear acceleration in the body reference frame in {}", GetLogFC()),
         [this]() { return GetAccelerationInBody(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("COGLinearAccelerationInWorld", "m/s2",
         fmt::format("COG body linear acceleration in the world reference frame in {}", GetLogFC()),
         [this]() { return GetCOGLinearAccelerationInWorld(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("COGLinearAccelerationInBody", "m/s2",
         fmt::format("COG body linear acceleration in the body reference frame in {}", GetLogFC()),
         [this]() { return GetCOGAccelerationInBody(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("AngularAccelerationInWorld", "rad/s2",
         fmt::format("body angular acceleration in the world reference frame in {}", GetLogFC()),
         [this]() { return GetAngularAccelerationInWorld(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("AngularAccelerationInBody", "rad/s2",
         fmt::format("body angular acceleration in the body reference frame in {}", GetLogFC()),
         [this]() { return GetAngularAccelerationInBody(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TotalExtForceInBody", "N",
         fmt::format("Total external force, expressed in body reference frame in {}", GetLogFC()),
         [this]() { return GetTotalExtForceInBody(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TotalExtTorqueInBodyAtCOG", "Nm",
         fmt::format("Total external torque at COG, expressed in body reference frame in {}", GetLogFC()),
         [this]() { return GetTotalExtTorqueInBodyAtCOG(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TotalExtForceInWorld", "N",
         fmt::format("Total external force, expressed in world reference frame in {}", GetLogFC()),
         [this]() { return GetTotalExtForceInWorld(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("TotalExtTorqueInWorldAtCOG", "Nm",
         fmt::format("Total external torque at COG, expressed in world reference frame in {}", GetLogFC()),
         [this]() { return GetTotalExtTorqueInWorldAtCOG(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("ContactForceInWorld", "N",
         fmt::format("Total contact force, expressed in world reference frame in {}", GetLogFC()),
         [this]() { return GetContactForceInWorld(GetLogFC()); });

  }

  FrBody::ForceContainer FrBody::GetForceList() const { return m_externalForces; }

  FrBody::NodeContainer FrBody::GetNodeList() const { return m_nodes; }


}  // end namespace frydom
