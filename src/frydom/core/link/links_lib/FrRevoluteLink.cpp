//
// Created by frongere on 23/01/19.
//

#include "FrRevoluteLink.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/core/common/FrNode.h"


namespace frydom {

    namespace internal {

        FrLinkMotorRotationSpeedBase::FrLinkMotorRotationSpeedBase(FrLinkBase_ *frydomLink) : m_frydomLink(frydomLink) {
            SetSpindleConstraint(chrono::ChLinkMotorRotation::SpindleConstraint::FREE);
        }

        void FrLinkMotorRotationSpeedBase::Initialize() {

            // Based on ChLinkMateGeneric::Initialize

            this->Body1 = m_frydomLink->GetBody1()->GetChronoBody().get();
            this->Body2 = m_frydomLink->GetBody2()->GetChronoBody().get();

            this->mask->SetTwoBodiesVariables(&Body1->Variables(), &Body2->Variables());

            this->frame1 = internal::FrFrame2ChFrame(m_frydomLink->GetNode1()->GetFrameWRT_COG_InBody());
            this->frame2 = internal::FrFrame2ChFrame(m_frydomLink->GetNode2()->GetFrameWRT_COG_InBody());
        }






    }  // end namespace frydom::internal







    FrRevoluteLink::FrRevoluteLink(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2,
                                   FrOffshoreSystem_ *system) : FrLink_(node1, node2, system) {
        m_chronoLink->SetLinkType(REVOLUTE);
    }

    void FrRevoluteLink::SetSpringDamper(double stiffness, double damping) {
        m_stiffness = stiffness;
        m_damping = damping;
    }

    void FrRevoluteLink::SetRestAngle(double restAngle) {
        m_frame2WRT1_reference.SetRotZ_RADIANS(restAngle, NWU);
        UpdateCache();
    }

    double FrRevoluteLink::GetRestAngle() const {
        return m_restAngle;
    }

    const Direction FrRevoluteLink::GetLinkAxisInWorld(FRAME_CONVENTION fc) const {
        return GetNode1()->GetFrameInWorld().GetZAxisInParent(fc);
    }

    double FrRevoluteLink::GetLinkAngle() const {
        return m_totalLinkAngle - m_restAngle;
    }

    double FrRevoluteLink::GetRelativeLinkAngle() const {
        return fmod(m_totalLinkAngle, MU_2PI) - m_restAngle;
    }

    int FrRevoluteLink::GetNbTurns() const {
        return int( (GetLinkAngle() - GetRelativeLinkAngle()) / MU_2PI );
    }

    double FrRevoluteLink::GetLinkAngularVelocity() const {
        return m_linkAngularVelocity;
    }

    double FrRevoluteLink::GetLinkAngularAcceleration() const {
        return m_linkAngularAcceleration;
    }

    double FrRevoluteLink::GetLinkTorque() const {
        return GetLinkTorqueOnBody2InFrame2AtOrigin2(NWU).GetMz();
    }

    double FrRevoluteLink::GetLinkPower() const {
        return GetLinkAngularVelocity() * GetLinkTorque();
    }

    void FrRevoluteLink::Initialize() {
        // Initialization of the constraint part
        FrLink_::Initialize();

        // Initialization of the motor part
        if (m_motor) {
            m_motor->Initialize();
        }

        // Log initialization
        l_message.SetNameAndDescription("RevoluteLink", "");
        l_message.AddCSVSerializer();
        l_message.AddField<double>("time", "s", "", &l_time);
        l_message.AddField<double>("total_angle", "deg", "", &l_angleDeg);
        l_message.AddField<double>("angVel", "rad/s", "", &m_linkAngularVelocity);
        l_message.AddField<double>("AnfAcc", "rad/s2", "", &m_linkAngularAcceleration);
        l_message.AddField<double>("Torque", "N.m", "", &l_torque);

        l_message.Initialize();
        l_message.Send();

    }

    void FrRevoluteLink::Update(double time) {

        FrLink_::Update(time);

        double lastRelativeAngle = GetRelativeLinkAngle() + m_restAngle; // Making it relative to x, not the rest angle
        double updatedRelativeAngle = GetUpdatedRelativeAngle();

        // TODO : voir a definir un RotationVector dans FrVector...
        // Computing the angle increment between current relative angle and the last relative angle to increment
        // the total link angle
        double angleIncrement;
        if (fabs(updatedRelativeAngle + MU_2PI - lastRelativeAngle) < fabs(updatedRelativeAngle - lastRelativeAngle)) {
            angleIncrement = updatedRelativeAngle + MU_2PI - lastRelativeAngle;
        } else if (fabs(updatedRelativeAngle - MU_2PI - lastRelativeAngle) < fabs(updatedRelativeAngle - lastRelativeAngle)) {
            angleIncrement = updatedRelativeAngle - MU_2PI - lastRelativeAngle;
        } else {
            angleIncrement = updatedRelativeAngle - lastRelativeAngle;
        }

        m_totalLinkAngle += angleIncrement;

        m_linkAngularVelocity = GetAngularVelocityOfMarker2WRTMarker1(NWU).GetWz();
        m_linkAngularAcceleration = GetAngularAccelerationOfMarker2WRTMarker1(NWU).GetWzp();

        UpdateForces(time);

    }

    void FrRevoluteLink::StepFinalize() {

        // Log
        l_time = m_system->GetTime();
        l_torque = GetLinkTorqueOnBody2InFrame1AtOrigin2(NWU).GetMz();
        l_angleDeg = m_totalLinkAngle * RAD2DEG;

        l_message.Serialize();
        l_message.Send();

        // Log
    }

    void FrRevoluteLink::UpdateForces(double time) {

        // Default spring damper force model
        Force force;
        Torque torque;

        torque.GetMz() = - m_stiffness * GetLinkAngle() - m_damping * GetLinkAngularVelocity();

        // Using force model from motor
        /*
         * TODO : si on a moteur force, on l'appelle ici et on ne prend pas en compte le spring damper...
         * Si on a un moteur, faut-il deconnecter le modele spring damper ??
         */

        // Set the link force
        SetLinkForceTorqueOnBody2InFrame2AtOrigin2(force, torque);
    }

    void FrRevoluteLink::MotorizeSpeed() {
//        m_motor = std::make_shared<internal::FrLinkMotorRotationSpeedBase>(this);
//
//        m_system->AddLink(m_motor);
//
//
//
//        // TODO : terminer




    }

    double FrRevoluteLink::GetUpdatedRelativeAngle() const {
        return mathutils::Normalize__PI_PI(m_chronoLink->c_frame2WRT1.GetRotation().GetRotationVector(NWU)[2]);
    }

    void FrRevoluteLink::UpdateCache() {
        // Updating the rest angle
        m_restAngle = mathutils::Normalize__PI_PI(m_frame2WRT1_reference.GetRotation().GetAngle());
        // TODO : ne pas prendre GetAngle mais la composante z de RotationVector

        // FIXME : attention si la liaison n'est pas resolue !!! Ca ne fonctionne pas
    }

    std::shared_ptr<FrRevoluteLink>
    make_revolute_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_ *system) {
        auto link = std::make_shared<FrRevoluteLink>(node1, node2, system);
        system->AddLink(link);
        return link;
    }



}  // end namespace frydom
