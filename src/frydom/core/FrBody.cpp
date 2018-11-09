//
// Created by frongere on 21/06/17.
//


#include "FrBody.h"
#include "FrNode.h"
#include "FrRotation.h"
#include "FrMatrix.h"

#include "chrono/assets/ChTriangleMeshShape.h"
#include "FrFrame.h"

#include "FrForce.h"

#include "frydom/environment/FrEnvironmentInc.h"
#include "frydom/environment/waves/FrFreeSurface.h"

#include <chrono/physics/ChLinkMotorLinearSpeed.h>  // FIXME : a retirer

#include "FrFunction.h"
#include "FrException.h"


namespace frydom {

    void FrBody::AddNode(std::shared_ptr<FrNode> node) {
        // Adding the node as a marker to the body
        AddMarker(node);
        // TODO: PUT Force related stuff here

    }

    std::shared_ptr<FrNode> FrBody::CreateNode() {

        // Creating the node and updating its position
        auto node = std::make_shared<FrNode>();
        node->SetBody(this);  // FIXME: a priori, c'est deja fait dans AddMarker lors de l'appele a AddNode... A retirer
        node->UpdateState();  // TODO: voir s'il est besoin d'appeler l'update...

        AddNode(node);
        return node;
    }

    std::shared_ptr<FrNode> FrBody::CreateNode(const chrono::ChVector<double> relpos) {

        auto node = CreateNode();

//        node->SetPos(relpos);  // TODO: Voir a utiliser ImposeRelPos tel que demande sur la liste chrono
        chrono::ChCoordsys<> coord;
        coord.pos = relpos;
        node->Impose_Rel_Coord(coord);
        node-> UpdateState();

        return node;
    }


    void FrBody::SetVisuMesh(std::shared_ptr<FrTriangleMeshConnected> mesh) {

        m_visu_mesh = mesh;

        auto shape = std::make_shared<chrono::ChTriangleMeshShape>();
        shape->SetMesh(*mesh);
        AddAsset(shape);

    }

    void FrBody::SetVisuMesh(std::string obj_filename) {
        auto mesh=std::make_shared<FrTriangleMeshConnected>();
        mesh->LoadWavefrontMesh(obj_filename);
        SetVisuMesh(mesh);
    }






























    /// REFACTORING ------------->>>>>>>>>>>>>>>


    #define DEFAULT_MAX_SPEED (float)10.
    #define DEFAULT_MAX_ROTATION_SPEED (float)(180.*DEG2RAD)

    namespace internal {

        _FrBodyBase::_FrBodyBase(FrBody_ *body) : chrono::ChBodyAuxRef(), m_frydomBody(body) {}

        void _FrBodyBase::SetupInitial() {
            m_frydomBody->Initialize();
        }

        void _FrBodyBase::Update(bool update_assets) {
            chrono::ChBodyAuxRef::Update(update_assets);
            m_frydomBody->Update();
        }

    }  // end namespace internal

    FrBody_::FrBody_() {
        m_chronoBody = std::make_shared<internal::_FrBodyBase>(this);
        m_chronoBody->SetMaxSpeed(DEFAULT_MAX_SPEED);
        m_chronoBody->SetMaxWvel(DEFAULT_MAX_ROTATION_SPEED);
    }

    FrOffshoreSystem_* FrBody_::GetSystem(){
        return m_system;
    }

    void FrBody_::SetName(const char *name) {
        m_chronoBody->SetName(name);
    }

    void FrBody_::SetBodyFixed(bool state) {
        m_chronoBody->SetBodyFixed(state);
    }

    void FrBody_::Initialize() {

        // Initializing forces
        auto forceIter = force_begin();
        for (; forceIter != force_end(); forceIter++) {
            (*forceIter)->Initialize();
        }

        // TODO : initialiser les logs

    }

    void FrBody_::StepFinalize() {
        // TODO
    }

    void FrBody_::Update() {
        // TODO
    }

    void FrBody_::SetSmoothContact() {
        auto materialSurface = std::make_shared<chrono::ChMaterialSurfaceSMC>();
        m_chronoBody->SetMaterialSurface(materialSurface);
        m_contactType = CONTACT_TYPE::SMOOTH_CONTACT;
    }

    void FrBody_::SetNonSmoothContact() {
        auto materialSurface = std::make_shared<chrono::ChMaterialSurfaceNSC>();
        m_chronoBody->SetMaterialSurface(materialSurface);
        m_contactType = CONTACT_TYPE::NONSMOOTH_CONTACT;
    }

    void FrBody_::SetContactMethod(CONTACT_TYPE contactType) {
        switch (contactType) {
            case CONTACT_TYPE::SMOOTH_CONTACT:
                SetSmoothContact();
                break;
            case CONTACT_TYPE::NONSMOOTH_CONTACT:
                SetNonSmoothContact();
                break;
        }
    }

    FrBody_::CONTACT_TYPE FrBody_::GetContactType() const {
        return m_contactType;
    }


    // Linear iterators

    FrBody_::ForceIter FrBody_::force_begin() {
        return m_externalForces.begin();
    }

    FrBody_::ConstForceIter FrBody_::force_begin() const {
        return m_externalForces.cbegin();
    }

    FrBody_::ForceIter FrBody_::force_end() {
        return m_externalForces.end();
    }

    FrBody_::ConstForceIter FrBody_::force_end() const {
        return m_externalForces.cend();
    }


    void FrBody_::AddMeshAsset(std::string obj_filename) {

        auto mesh = std::make_shared<FrTriangleMeshConnected>();
        mesh->LoadWavefrontMesh(obj_filename);
        AddMeshAsset(mesh);
    }

    void FrBody_::AddMeshAsset(std::shared_ptr<frydom::FrTriangleMeshConnected> mesh) {
        auto shape = std::make_shared<chrono::ChTriangleMeshShape>();
        shape->SetMesh(*mesh);
        m_chronoBody->AddAsset(shape);
    }

    void FrBody_::SetColor(NAMED_COLOR colorName) {
        SetColor(FrColor(colorName));
    }

    void FrBody_::SetColor(const FrColor& color) {
        auto colorAsset = std::make_shared<chrono::ChColorAsset>(
                chrono::ChColor(color.R, color.G, color.B));
        m_chronoBody->AddAsset(colorAsset);
    }




    void FrBody_::ConstainDOF(bool cx, bool cy, bool cz, bool crx, bool cry, bool crz) {

//        auto link = std::make_shared<>()

    }

    void FrBody_::ConstrainInVx(double Vx) {

        auto motor = std::make_shared<chrono::ChLinkMotorLinearSpeed>();

        auto frame = chrono::ChFrame<double>();


        motor->Initialize(m_chronoBody, GetSystem()->GetEnvironment()->GetFreeSurface()->m_body->m_chronoBody, frame);

        m_system->AddLink(motor);


        auto rampConst = std::make_shared<FrFunction>(Vx);

        motor->SetSpeedFunction(rampConst);



    }




    double FrBody_::GetMass() const {
        return m_chronoBody->GetMass();
    }


    FrInertiaTensor_ FrBody_::GetInertiaParams() const {
        double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
        SplitMatrix33IntoCoeffs(internal::ChMatrix33ToMatrix33(m_chronoBody->GetInertia()),
                Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz);

        return {GetMass(), Ixx, Iyy, Izz, Ixy, Ixz, Iyz, FrFrame_(GetCOG(NWU), FrRotation_(), NWU), NWU};
    }

    void FrBody_::SetInertiaParams(const FrInertiaTensor_& inertia) {

        m_chronoBody->SetMass(inertia.GetMass());

        SetCOG(inertia.GetCOGPosition(NWU), NWU);

        double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
        inertia.GetInertiaCoeffs(Ixx, Iyy, Izz, Ixy, Ixz, Iyz, NWU);

        m_chronoBody->SetInertiaXX(chrono::ChVector<double>(Ixx, Iyy, Izz));
        m_chronoBody->SetInertiaXY(chrono::ChVector<double>(Ixy, Ixz, Iyz));

    }

    void FrBody_::SetInertiaParams(double mass,
                          double Ixx, double Iyy, double Izz,
                          double Ixy, double Ixz, double Iyz,
                          const FrFrame_& coeffsFrame,
                          const Position& cogPosition,
                          FRAME_CONVENTION fc) {
        SetInertiaParams(FrInertiaTensor_(mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, coeffsFrame, cogPosition, fc));
    }

    void FrBody_::SetInertiaParams(double mass,
                          double Ixx, double Iyy, double Izz,
                          double Ixy, double Ixz, double Iyz,
                          const FrFrame_& cogFrame,
                          FRAME_CONVENTION fc) {
        SetInertiaParams(FrInertiaTensor_(mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, cogFrame, fc));
    }

    void FrBody_::SetCollide(bool isColliding) {
        m_chronoBody->SetCollide(isColliding);
    }

    void FrBody_::AddBoxShape(double xSize, double ySize, double zSize) {
        auto shape = std::make_shared<chrono::ChBoxShape>();
        shape->GetBoxGeometry().SetLengths(chrono::ChVector<double>(xSize, ySize, zSize));
        m_chronoBody->AddAsset(shape);
    }

    void FrBody_::AddCylinderShape(double radius, double height) {
        auto shape = std::make_shared<chrono::ChCylinderShape>();
        shape->GetCylinderGeometry().p1 = chrono::ChVector<double>(0., -height*0.5, 0.);
        shape->GetCylinderGeometry().p2 = chrono::ChVector<double>(0.,  height*0.5, 0.);
        shape->GetCylinderGeometry().rad = radius;
        m_chronoBody->AddAsset(shape);
    }

    void FrBody_::AddSphereShape(double radius) {
        auto shape = std::make_shared<chrono::ChSphereShape>();
        shape->GetSphereGeometry().rad = radius;
        m_chronoBody->AddAsset(shape);
    }

    void FrBody_::ActivateSpeedLimits(bool activate) {
        m_chronoBody->SetLimitSpeed(activate);
    }

    void FrBody_::SetMaxSpeed(double maxSpeed_ms) {
        m_chronoBody->SetMaxSpeed((float)maxSpeed_ms);
    }

    void FrBody_::SetMaxRotationSpeed(double wMax_rads) {
        m_chronoBody->SetMaxWvel((float)wMax_rads);
    }

    void FrBody_::RemoveGravity(bool val) { // TODO : ajouter la force d'accumulation a l'initialisation --> cas ou le systeme n'a pas encore ete precise pour la gravite...
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

    void FrBody_::AddExternalForce(std::shared_ptr<frydom::FrForce_> force) {
        m_chronoBody->AddForce(force->GetChronoForce());  // FrBody_ is a friend class of FrForce_

        force->m_body = this;
        m_externalForces.push_back(force);
    }

    void FrBody_::RemoveExternalForce(std::shared_ptr<FrForce_> force) {
        m_chronoBody->RemoveForce(force->GetChronoForce());

        m_externalForces.erase(
                std::find<std::vector<std::shared_ptr<FrForce_>>::iterator>(m_externalForces.begin(), m_externalForces.end(), force));

        force->m_body = nullptr;
    }

    void FrBody_::RemoveAllForces() {
        m_chronoBody->RemoveAllForces();
        for (auto forceIter=force_begin(); forceIter!=force_end(); forceIter++) {
            (*forceIter)->m_body = nullptr;
        }
        m_externalForces.clear();
    }



    // Nodes

    std::shared_ptr<FrNode_> FrBody_::NewNode(const frydom::FrFrame_ &localFrame) {
        return std::make_shared<FrNode_>(this, localFrame);
    }

    std::shared_ptr<FrNode_> FrBody_::NewNode(const frydom::Position &localPosition) {
        return std::make_shared<FrNode_>(this, localPosition);
    }

    std::shared_ptr<FrNode_> FrBody_::NewNode(double x, double y, double z) {
        return std::make_shared<FrNode_>(this, Position(x, y, z));
    }


    void FrBody_::SetMass(double mass) {
        m_chronoBody->SetMass(mass);
    }

    void FrBody_::SetCOG(const Position& bodyPos, FRAME_CONVENTION fc) {
        FrFrame_ cogFrame;
        cogFrame.SetPosition(bodyPos, fc);
        m_chronoBody->SetFrame_COG_to_REF(internal::Fr2ChFrame(cogFrame));
    }

    Position FrBody_::GetCOG(FRAME_CONVENTION fc) const {
        Position cogPos = internal::ChVectorToVector3d<Position>(m_chronoBody->GetFrame_COG_to_REF().GetPos()); // In NWU
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(cogPos);
        return cogPos;
    }

    Position FrBody_::GetPosition(FRAME_CONVENTION fc) const {
        return internal::ChVectorToVector3d<Position>(m_chronoBody->GetFrame_REF_to_abs().GetPos());
    }

    void FrBody_::SetPosition(const Position &worldPos, FRAME_CONVENTION fc) {
        auto bodyFrame = GetFrame();
        bodyFrame.SetPosition(worldPos, fc);
        m_chronoBody->SetFrame_REF_to_abs(internal::Fr2ChFrame(bodyFrame));
        // Traiter update
    }

    FrRotation_ FrBody_::GetRotation() const {
        return FrRotation_(GetQuaternion());
    }

    void FrBody_::SetRotation(const FrRotation_ &rotation) {
        SetRotation(rotation.GetQuaternion());
    }

    FrQuaternion_ FrBody_::GetQuaternion() const {
        return internal::Ch2FrQuaternion(m_chronoBody->GetRot());
    }

    void FrBody_::SetRotation(const FrQuaternion_ &quaternion) {
        m_chronoBody->SetRot(internal::Fr2ChQuaternion(quaternion));
    }

    FrFrame_ FrBody_::GetFrame() const {
        FrFrame_ bodyRefFrame;
        bodyRefFrame.SetPosition(GetPosition(NWU), NWU);
        bodyRefFrame.SetRotation(GetQuaternion());
        return bodyRefFrame;
    }

    void FrBody_::SetFrame(const FrFrame_ &worldFrame) {
        SetPosition(worldFrame.GetPosition(NWU), NWU);
        SetRotation(worldFrame.GetQuaternion());
    }

    FrFrame_ FrBody_::GetFrameAtPoint(const Position& bodyPoint, FRAME_CONVENTION fc) {
        FrFrame_ pointFrame;
        pointFrame.SetPosition(GetPointPositionInWorld(bodyPoint, fc), fc);
        pointFrame.SetRotation(GetQuaternion());
        return pointFrame;
    }

    FrFrame_ FrBody_::GetFrameAtCOG(FRAME_CONVENTION fc) {
        return GetFrameAtPoint(GetCOG(fc), fc);
    }

    Position FrBody_::GetPointPositionInWorld(const Position &bodyPos, FRAME_CONVENTION fc) const {
        return GetPosition(fc) + ProjectVectorInWorld<Position>(bodyPos, fc);
    }

    Position FrBody_::GetPointPositionInBody(const Position &worldPos, FRAME_CONVENTION fc) const {
        return ProjectVectorInBody<Position>(worldPos - GetPosition(fc), fc);
    }

    Position FrBody_::GetCOGPositionInWorld(FRAME_CONVENTION fc) const {
        Position cogPos = internal::ChVectorToVector3d<Position>(m_chronoBody->GetPos());
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(cogPos);
        return cogPos;
    }

    void FrBody_::SetPositionOfBodyPoint(const Position &bodyPoint, const Position &worldPos, FRAME_CONVENTION fc) {
        _SetPointPosition(bodyPoint, BODY, worldPos, WORLD, fc);
    }

//    void FrBody_::SetPositionOfCOG(const Position &worldPos, FRAME_CONVENTION fc) {
//        _SetPointPosition(GetCOG(fc), BODY, worldPos, WORLD, fc);
//    }

    void FrBody_::TranslateInWorld(const Position &worldTranslation, FRAME_CONVENTION fc) {
        _Translate(worldTranslation, WORLD, fc);
    }

    void FrBody_::TranslateInBody(const Position &bodyTranslation, FRAME_CONVENTION fc) {
        _Translate(bodyTranslation, BODY, fc);
    }

    void FrBody_::_Translate(const Position &translation, FRAME translationFrame, FRAME_CONVENTION fc) {
        auto refFrame = GetFrame();

        if (translationFrame == BODY) {
            refFrame.SetPosition(refFrame.GetPosition(fc) + ProjectVectorInWorld<Position>(translation, fc), fc);
        } else {
            refFrame.SetPosition(refFrame.GetPosition(fc) + translation, fc);
        }

        m_chronoBody->SetFrame_REF_to_abs(internal::Fr2ChFrame(refFrame));
    }

    void FrBody_::Rotate(const FrRotation_ &relRotation) {
        SetRotation(GetRotation() * relRotation);
    }

    void FrBody_::Rotate(const FrQuaternion_ &relQuaternion) {
        SetRotation(GetQuaternion() * relQuaternion);
    }

    void FrBody_::RotateAroundPointInWorld(const FrRotation_& rot, const Position& worldPos, FRAME_CONVENTION fc) {
        RotateAroundPointInWorld(rot.GetQuaternion(), worldPos, fc);
    }

    void FrBody_::RotateAroundPointInBody(const FrRotation_& rot, const Position& bodyPos, FRAME_CONVENTION fc) {
        RotateAroundPointInBody(rot.GetQuaternion(), bodyPos, fc);
    }

    void FrBody_::RotateAroundPointInWorld(const FrQuaternion_& rot, const Position& worldPos, FRAME_CONVENTION fc) {
        Position bodyPos = GetPointPositionInBody(worldPos, fc);
        Rotate(rot);
        SetPositionOfBodyPoint(bodyPos, worldPos, fc);
    }

    void FrBody_::RotateAroundPointInBody(const FrQuaternion_& rot, const Position& bodyPos, FRAME_CONVENTION fc) {
        Position worldPos = GetPointPositionInWorld(bodyPos, fc);
        Rotate(rot);
        SetPositionOfBodyPoint(bodyPos, worldPos, fc);
    }

    void FrBody_::RotateAroundCOG(const FrRotation_& rot, FRAME_CONVENTION fc) {
        RotateAroundPointInBody(rot, GetCOG(fc), fc);
    }

    void FrBody_::RotateAroundCOG(const FrQuaternion_& rot, FRAME_CONVENTION fc) {
        RotateAroundPointInBody(rot, GetCOG(fc), fc);
    }


//    void FrBody_::_SetVelocityAtPoint(const Position& point, FRAME pointFrame, FRAME pointFrameExpr,
//                                      const Velocity& vel, FRAME velFrame,
//                                      const AngularVelocity& angVel, FRAME angVelFrame, FRAME_CONVENTION fc) {
//
//        if (velFrame == BODY) {
//
//        }
//    }
//
//    void FrBody_::SetVelocityInWorld(const Velocity &worldVel, FRAME_CONVENTION fc) {
//
//    }
//
//    void FrBody_::SetVelocityInBody(const Velocity &bodyVel, FRAME_CONVENTION fc) {
//
//    }
//
    Velocity FrBody_::GetVelocityInWorld(FRAME_CONVENTION fc) const {
        Velocity bodyVel = internal::ChVectorToVector3d<Velocity>(m_chronoBody->GetFrame_REF_to_abs().GetPos_dt());
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(bodyVel);
        return bodyVel;
    }

    Velocity FrBody_::GetVelocityInBody(FRAME_CONVENTION fc) const {
        return ProjectVectorInBody<Velocity>(GetVelocityInWorld(fc), fc);
    }

    void FrBody_::SetCOGVelocityInWorld(const Velocity &worldVel, FRAME_CONVENTION fc) {
        auto worldVelTmp = worldVel;
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(worldVelTmp);
        chrono::ChCoordsys<double> coord;
        coord.pos = internal::Vector3dToChVector(worldVelTmp);
        m_chronoBody->SetCoord_dt(coord);
    }

    void FrBody_::SetCOGVelocityInBody(const Velocity &bodyVel, FRAME_CONVENTION fc) {
        SetCOGVelocityInWorld(ProjectVectorInWorld(bodyVel, fc), fc);
    }

    Velocity FrBody_::GetCOGVelocityInWorld(FRAME_CONVENTION fc) const {
        Velocity cogVel = internal::ChVectorToVector3d<Velocity>(m_chronoBody->GetCoord_dt().pos); // In NWU
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(cogVel);
        return cogVel;
    }

    Velocity FrBody_::GetCOGVelocityInBody(FRAME_CONVENTION fc) const {
        return ProjectVectorInBody<Velocity>(GetCOGVelocityInWorld(fc), fc);
    }

//    void FrBody_::SetAccelerationInWorld(const Velocity &worldVel, FRAME_CONVENTION fc) {
//
//    }
//
//    void FrBody_::SetAccelerationInBody(const Velocity &bodyVel, FRAME_CONVENTION fc) {
//
//    }
//
//    void FrBody_::GetAccelerationInWorld(const Velocity &worldVel, FRAME_CONVENTION fc) {
//
//    }
//
//    void FrBody_::GetAccelerationInBody(const Velocity &bodyVel, FRAME_CONVENTION fc) {
//
//    }

    void FrBody_::SetCOGAccelerationInWorld(const Acceleration &worldAcc, FRAME_CONVENTION fc) {
        auto worldAccTmp = worldAcc;
        if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(worldAccTmp);
        chrono::ChCoordsys<double> coord;
        coord.pos = internal::Vector3dToChVector(worldAccTmp);
        m_chronoBody->SetCoord_dtdt(coord);
    }

    void FrBody_::SetCOGAccelerationInBody(const Acceleration &bodyAcc, FRAME_CONVENTION fc) {
        SetCOGAccelerationInWorld(ProjectVectorInWorld<Acceleration>(bodyAcc, fc), fc);
    }

    Acceleration FrBody_::GetCOGAccelerationInWorld(FRAME_CONVENTION fc) const {
        Acceleration cogAcc = internal::ChVectorToVector3d<Acceleration>(m_chronoBody->GetCoord_dtdt().pos); // In NWU
        if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(cogAcc);
        return cogAcc;
    }

    Acceleration FrBody_::GetCOGAccelerationInBody(FRAME_CONVENTION fc) const {
        return ProjectVectorInBody<Acceleration>(GetCOGAccelerationInWorld(fc), fc);
    }

    void FrBody_::SetAngularVelocityInWorld(const AngularVelocity &worldAngVel, FRAME_CONVENTION fc) {
        auto worldAngVelTmp = worldAngVel;
        if (IsNED(fc)) internal::SwapFrameConvention<AngularVelocity>(worldAngVelTmp);
        m_chronoBody->SetWvel_par(internal::Vector3dToChVector(worldAngVelTmp));
    }

    void FrBody_::SetAngularVelocityInBody(const AngularVelocity &bodyAngVel, FRAME_CONVENTION fc) {
        SetAngularVelocityInWorld(ProjectVectorInWorld(bodyAngVel, fc), fc);
    }

    AngularVelocity FrBody_::GetAngularVelocityInWorld(FRAME_CONVENTION fc) const {
        AngularVelocity angVel = internal::ChVectorToVector3d<AngularVelocity>(m_chronoBody->GetWvel_par());
        if (IsNED(fc)) internal::SwapFrameConvention<AngularVelocity>(angVel);
        return angVel;
    }

    AngularVelocity FrBody_::GetAngularVelocityInBody(FRAME_CONVENTION fc) const {
        return ProjectVectorInBody<AngularVelocity>(GetAngularVelocityInWorld(fc), fc);
    }

    void FrBody_::SetAngularAccelerationInWorld(const AngularAcceleration &worldAngAcc, FRAME_CONVENTION fc) {
        auto worldAngAccTmp = worldAngAcc;
        if (IsNED(fc)) internal::SwapFrameConvention<AngularVelocity>(worldAngAccTmp);
        auto chronoAngAcc = internal::Vector3dToChVector(worldAngAccTmp);
        m_chronoBody->SetWacc_par(chronoAngAcc); // FIXME : dans chrono, l'argument d'entree n'est pas const... -> fix Chrono
    }

    void FrBody_::SetAngularAccelerationInBody(const AngularAcceleration &bodyAngAcc, FRAME_CONVENTION fc) {
        SetAngularAccelerationInWorld(ProjectVectorInWorld(bodyAngAcc, fc), fc);
    }

    AngularAcceleration FrBody_::GetAngularAccelerationInWorld(FRAME_CONVENTION fc) const {
        AngularAcceleration angAcc = internal::ChVectorToVector3d<AngularAcceleration>(m_chronoBody->GetWacc_par());
        if (IsNED(fc)) internal::SwapFrameConvention<AngularAcceleration>(angAcc);
        return angAcc;
    }

    AngularAcceleration FrBody_::GetAngularAccelerationInBody(FRAME_CONVENTION fc) const {
        return ProjectVectorInBody(GetAngularAccelerationInWorld(fc), fc);
    }

    Velocity FrBody_::GetVelocityInWorldAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {
        Position bodyPoint = GetPointPositionInBody(worldPoint, fc);
        return GetVelocityInWorldAtPointInBody(bodyPoint, fc);
    }

    Velocity FrBody_::GetVelocityInWorldAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {
        return ProjectVectorInWorld<Velocity>(GetVelocityInBodyAtPointInBody(bodyPoint, fc), fc);
    }

    Velocity FrBody_::GetVelocityInBodyAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {
        Position bodyPoint = GetPointPositionInBody(worldPoint, fc);
        return GetVelocityInBodyAtPointInBody(bodyPoint, fc);
    }

    Velocity FrBody_::GetVelocityInBodyAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {
        Velocity bodyVel = GetVelocityInBody(fc);
        AngularVelocity bodyAngVel = GetAngularVelocityInBody(fc);
        Velocity pointVel = bodyVel + bodyAngVel.cross(bodyPoint);
    }

    Acceleration FrBody_::GetAccelerationInWorldAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {
        auto bodyPoint = GetPointPositionInBody(worldPoint, fc);
        return GetAccelerationInWorldAtPointInBody(bodyPoint, fc);
    }

    Acceleration FrBody_::GetAccelerationInWorldAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {
        auto bodyPointTmp = bodyPoint;
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(bodyPointTmp);

        Acceleration pointAcc = internal::ChVectorToVector3d<Acceleration>(
                m_chronoBody->PointAccelerationLocalToParent(internal::Vector3dToChVector(bodyPointTmp - GetCOG(NWU)))
                );

        if (IsNED(fc)) internal::SwapFrameConvention<Acceleration>(pointAcc);
        return pointAcc;
    }

    Acceleration FrBody_::GetAccelerationInBodyAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {
        GetAccelerationInBodyAtPointInBody(GetPointPositionInBody(worldPoint, fc), fc);
    }

    Acceleration FrBody_::GetAccelerationInBodyAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {
        return ProjectVectorInBody(GetAccelerationInWorldAtPointInBody(bodyPoint, fc), fc);
    }

    
    void FrBody_::SetGeneralizedVelocityInWorldAtPointInWorld(const Position &worldPoint, const Velocity &worldVel,
                                                              const AngularVelocity &worldAngVel, FRAME_CONVENTION fc) {
        Position bodyPoint = GetPointPositionInBody(worldPoint, fc);
        SetGeneralizedVelocityInWorldAtPointInBody(bodyPoint, worldVel, worldAngVel, fc);
    }

    void FrBody_::SetGeneralizedVelocityInWorldAtPointInBody(const Position &bodyPoint, const Velocity &worldVel,
                                                             const AngularVelocity &worldAngVel, FRAME_CONVENTION fc) {
        Velocity bodyVel = ProjectVectorInBody<Velocity>(worldVel, fc);
        AngularVelocity bodyAngVel = ProjectVectorInBody<AngularVelocity>(worldAngVel, fc);
        SetGeneralizedVelocityInBodyAtPointInBody(bodyPoint, bodyVel, bodyAngVel, fc);
    }

    void FrBody_::SetGeneralizedVelocityInBodyAtPointInWorld(const Position &worldPoint, const Velocity &bodyVel,
                                                             const AngularVelocity &bodyAngVel, FRAME_CONVENTION fc) {
        Position bodyPoint = GetPointPositionInBody(worldPoint, fc);
        SetGeneralizedVelocityInBodyAtPointInBody(bodyPoint, bodyVel, bodyAngVel, fc);
    }

    void FrBody_::SetGeneralizedVelocityInBodyAtPointInBody(const Position &bodyPoint, const Velocity &bodyVel,
                                                            const AngularVelocity &bodyAngVel, FRAME_CONVENTION fc) {
        SetAngularVelocityInBody(bodyAngVel, fc);
        Position PG = GetCOG(fc) - bodyPoint;
        Velocity cogVel = bodyVel + bodyAngVel.cross(PG);
        SetCOGVelocityInBody(cogVel, fc);
    }







    void
    FrBody_::SetGeneralizedAccelerationInWorldAtPointInWorld(const Position &worldPoint, const Acceleration &worldAcc,
                                                             const AngularAcceleration &worldAngAcc, FRAME_CONVENTION fc) {


        // Voir la methode TransformLocaToParent() de ChFrameMoving pour le transport d'acceleration de point

//        // pos_dtdt
//        parent.coord_dtdt.pos =
//                PointAccelerationLocalToParent(local.coord.pos, local.coord_dt.pos, local.coord_dtdt.pos);
//
//        // rot_dt
//        parent.coord_dt.rot = coord_dt.rot % local.coord.rot + this->coord.rot % local.coord_dt.rot;
//
//        // rot_dtdt
//        parent.coord_dtdt.rot = coord_dtdt.rot % local.coord.rot + (coord_dt.rot % local.coord_dt.rot) * 2 +
//                                this->coord.rot % local.coord_dtdt.rot;



//      PointAccelerationLocalToParent :

        // Calule l'acceleration d'un point fixe au corps odnt les coords sont donnees en local au corps. L'acceleration
        // est exprimee dans le repere world

//        return coord_dtdt.pos +
//               ((coord_dtdt.rot % ChQuaternion<Real>(0, localpos) % this->coord.rot.GetConjugate()).GetVector() * 2) +
//               ((coord_dt.rot % ChQuaternion<Real>(0, localpos) % coord_dt.rot.GetConjugate()).GetVector() * 2);

        // Formule :
        // Acc_P_world = Acc_G_world + bodyQuat_pp *


    }

    void
    FrBody_::SetGeneralizedAccelerationInWorldAtPointInBody(const Position &bodyPoint, const Acceleration &worldAcc,
                                                            const AngularAcceleration &worldAngAcc, FRAME_CONVENTION fc) {

    }

    void
    FrBody_::SetGeneralizedAccelerationInBodyAtPointInWorld(const Position &worldPoint, const Acceleration &bodyAcc,
                                                            const AngularAcceleration &bodyAngAcc, FRAME_CONVENTION fc) {

    }

    void FrBody_::SetGeneralizedAccelerationInBodyAtPointInBody(const Position &bodyPoint, const Acceleration &bodyAcc,
                                                                const AngularAcceleration &bodyAngAcc, FRAME_CONVENTION fc) {

    }

//    Velocity FrBody_::GetApparentVelocityInWorldAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) {
//        return Velocity();
//    }
//
//    Velocity FrBody_::GetApparentVelocityInWorldAtPointInWorld(const Position &bodyPoint, FRAME_CONVENTION fc) {
//        return Velocity();
//    }
//
//    Velocity FrBody_::GetApparentVelocityInBodyAtPointInBody(const Position &worldPoint, FRAME_CONVENTION fc) {
//        return Velocity();
//    }
//
//    Velocity FrBody_::GetApparentVelocityInBodyAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) {
//        return Velocity();
//    }
//
//    double FrBody_::GetApparentAngle(FLUID_TYPE ft, FRAME_CONVENTION fc) {
//        return 0;
//    }



    void FrBody_::_SetPointPosition(const Position& point, FRAME pointFrame, const Position& pos, FRAME posFrame, FRAME_CONVENTION fc) {

        Position WB = GetPosition(fc);

        Position WP;
        if (pointFrame == BODY) {
            // we have BP
            WP = GetPointPositionInWorld(point, fc);
        } else {
            WP = point;
        }

        Position WT;
        if (posFrame == BODY) {
            // we have BT
            WT = GetPointPositionInWorld(pos, fc);
        } else {
            WT = pos;
        }

        Position PT = WT - WP;

        TranslateInWorld(PT, fc);

    }


}  // end namespace frydom