//
// Created by camille on 20/11/18.
//

#include "FrEquilibriumFrame.h"

namespace frydom {


    // ---------------------------------------------------------------------
    // Equilibrium frame
    // ---------------------------------------------------------------------

    void FrEquilibriumFrame_::SetVelocityInWorld(const Velocity& velocity, FRAME_CONVENTION fc) {
        if(IsNED(fc)) internal::SwapFrameConvention(velocity);
        m_velocity = velocity;
        m_initSpeedFromBody = false;
    }

    void FrEquilibriumFrame_::SetVelocityInFrame(const Velocity& frameVel) {
        auto worldVel = ProjectVectorInParent(frameVel);
        this->SetVelocityInWorld(worldVel, NWU);
        m_initSpeedFromBody = false;
    }

    void FrEquilibriumFrame_::SetAngularVelocityAroundZ(const double &angularVelocity, FRAME_CONVENTION fc) {
        m_angularVelocity = angularVelocity;
        if(IsNED(fc))  { m_angularVelocity = -m_angularVelocity; }
        m_initSpeedFromBody = false;
    }

    void FrEquilibriumFrame_::SetBody(FrBody_* body, bool initPos) {
        m_body = body;
        m_initPositionFromBody = initPos;
    }

    Velocity FrEquilibriumFrame_::GetVelocityInWorld(FRAME_CONVENTION fc) const {
        Velocity velocity = m_velocity;
        if (IsNED(fc)) internal::SwapFrameConvention(velocity);
        return velocity;
    }

    Velocity FrEquilibriumFrame_::GetVelocityInFrame() const {
        return ProjectVectorParentInFrame<Velocity>(m_velocity);
    }

    double FrEquilibriumFrame_::GetAngularVelocityAroundZ(FRAME_CONVENTION fc) const {
        double result = m_angularVelocity;
        if (IsNED(fc)) { result = -result; }
        return result;
    }

    AngularVelocity FrEquilibriumFrame_::GetAngularVelocity(FRAME_CONVENTION fc) const {
        auto wvel = GetAngularVelocityAroundZ(fc);
        return AngularVelocity(0., 0., wvel);
    }

    void FrEquilibriumFrame_::SetPositionToBodyPosition() {
        this->SetPosition(m_body->GetCOGPositionInWorld(NWU), NWU);
        this->SetRotation(m_body->GetRotation());
        m_initPositionFromBody = false;
    }

    void FrEquilibriumFrame_::SetVelocityToBodyVelocity() {
        m_velocity = m_body->GetCOGVelocityInWorld(NWU);
        m_angularVelocity = 0.;
        m_initSpeedFromBody = false;
    }

    void FrEquilibriumFrame_::Initialize() {

        if(!m_body) { throw FrException("error : the body is not defined in equilibrium frame"); }

        if (m_initPositionFromBody) this->SetPositionToBodyPosition();
        if (m_initSpeedFromBody) this->SetVelocityToBodyVelocity();
    }

    // -----------------------------------------------------------------------
    // Equilibrium frame with spring damping restoring force
    // -----------------------------------------------------------------------

    FrEqFrameSpringDamping_::FrEqFrameSpringDamping_(FrBody_* body, double T0, double psi, bool initPos)
        : FrEquilibriumFrame_(body, initPos) { this->SetSpringDamping(T0, psi); }

    FrEqFrameSpringDamping_::FrEqFrameSpringDamping_(const Position &pos, const FrRotation_ &rotation,
                                                     FRAME_CONVENTION fc, FrBody_* body, double T0, double psi)
            : FrEquilibriumFrame_(pos, rotation, fc, body) { this->SetSpringDamping(T0, psi); }

    FrEqFrameSpringDamping_::FrEqFrameSpringDamping_(const Position &pos, const FrUnitQuaternion_& quaternion,
                                                     FRAME_CONVENTION fc, FrBody_* body, double T0, double psi)
            : FrEquilibriumFrame_(pos, quaternion, fc, body) { this->SetSpringDamping(T0, psi); }

    FrEqFrameSpringDamping_::FrEqFrameSpringDamping_(const FrFrame_& otherFrame, FrBody_* body, double T0, double psi)
            : FrEquilibriumFrame_(otherFrame, body) { this->SetSpringDamping(T0, psi); }

    void FrEqFrameSpringDamping_::SetSpringDamping(const double T0, const double psi) {

        m_w0 = 2.*M_PI / T0;
        m_psi = psi;

        m_damping = 2. * m_psi * m_w0;
        m_stiffness = m_w0 * m_w0;
    }

    void FrEqFrameSpringDamping_::Update(double time) {

        if (std::abs(time - m_prevTime) < FLT_EPSILON) return;

        auto bodyPosition = m_body->GetCOGPositionInWorld(NWU);
        auto bodyVelocity = m_body->GetCOGVelocityInWorld(NWU);
        auto position = GetPosition(NWU);

        Force force;
        force = (bodyPosition - position) * m_stiffness + (bodyVelocity - m_velocity) * m_damping;
        force.GetFz() = 0.;

        double temp1, temp2;
        double bodyPsi, psi;
        GetRotation().GetCardanAngles_RADIANS(temp1, temp2, psi, NWU);
        m_body->GetRotation().GetCardanAngles_RADIANS(temp1, temp2, bodyPsi, NWU);
        auto bodyAngularVelocity = m_body->GetAngularVelocityInWorld(NWU).GetWz();

        double torque;
        torque = (bodyPsi - psi) * m_stiffness + (bodyAngularVelocity - m_angularVelocity) * m_damping;

        m_velocity += force * (time - m_prevTime);
        position += m_velocity * (time - m_prevTime);

        m_angularVelocity += torque * (time - m_prevTime);

        this->SetPosition(position, NWU);
        SetRotation( this->GetRotation().RotZ_RADIANS(m_angularVelocity * (time - m_prevTime), NWU) );

        m_prevTime = time;
    }

    // ----------------------------------------------------------------
    // Equilibrium frame with updated mean velocity
    // ----------------------------------------------------------------

    FrEqFrameMeanMotion_::FrEqFrameMeanMotion_(const Position &pos, const FrRotation_ &rotation, FRAME_CONVENTION fc,
                                               FrBody_* body, double timePersistence, double timeStep)
            : FrEquilibriumFrame_(pos, rotation, fc, body) { this->SetRecorders(timePersistence, timeStep); }

    FrEqFrameMeanMotion_::FrEqFrameMeanMotion_(const Position &pos, const FrUnitQuaternion_ &quaternion, FRAME_CONVENTION fc,
                                               FrBody_* body, double timePersistence, double timeStep)
            : FrEquilibriumFrame_(pos, quaternion, fc, body) { this->SetRecorders(timePersistence, timeStep); }

    FrEqFrameMeanMotion_::FrEqFrameMeanMotion_(const FrFrame_ &otherFrame, FrBody_* body, double timePersistence, double timeStep)
            : FrEquilibriumFrame_(otherFrame, body) { this->SetRecorders(timePersistence, timeStep); }

    FrEqFrameMeanMotion_::FrEqFrameMeanMotion_(FrBody_ *body, double timePersistence, double timeStep, bool initPos)
    : FrEquilibriumFrame_(body, initPos) { this->SetRecorders(timePersistence, timeStep); }


    void FrEqFrameMeanMotion_::SetRecorders(double timePersistence, double timeStep) {
        m_TrSpeedRec = std::make_unique<FrTimeRecorder_<Velocity>>(timePersistence, timeStep);
        m_TrSpeedRec->Initialize();
        m_AglSpeedRec = std::make_unique<FrTimeRecorder_<double>>(timePersistence, timeStep);
        m_AglSpeedRec->Initialize();
    }

    void FrEqFrameMeanMotion_::SetPositionCorrection(double timePersistence, double timeStep,
                                                     double posCoeff, double angleCoeff) {
        m_ErrPositionRec = std::make_unique<FrTimeRecorder_<Position>>(timePersistence, timeStep);
        m_ErrPositionRec->Initialize();
        m_ErrAngleRec = std::make_unique<FrTimeRecorder_<double>>(timePersistence, timeStep);
        m_ErrAngleRec->Initialize();
        m_errPosCoeff = posCoeff;
        m_errAngleCoeff = angleCoeff;
    }

    void FrEqFrameMeanMotion_::Update(double time) {

        if (std::abs(time - m_prevTime) < FLT_EPSILON) return;

        m_TrSpeedRec->Record(time, m_body->GetCOGVelocityInWorld(NWU));
        m_AglSpeedRec->Record(time, m_body->GetAngularVelocityInWorld(NWU).GetWz());

        m_velocity = m_TrSpeedRec->GetMean();
        m_angularVelocity = m_AglSpeedRec->GetMean();

        auto position = GetPosition(NWU);
        position += m_velocity * (time - m_prevTime);

        auto angle = m_angularVelocity * (time - m_prevTime);

        if (m_ErrPositionRec and m_ErrAngleRec) {

            m_ErrPositionRec->Record(time, m_body->GetCOGPositionInWorld(NWU) - GetPosition(NWU));
            auto errMeanPosition = m_ErrPositionRec->GetMean();
            position += errMeanPosition * m_errPosCoeff;

            double temp1, temp2, bodyAngle, frameAngle;
            m_body->GetRotation().GetCardanAngles_RADIANS(temp1, temp2, bodyAngle, NWU);
            this->GetRotation().GetCardanAngles_RADIANS(temp1, temp2, frameAngle, NWU);
            m_ErrAngleRec->Record(time, bodyAngle - frameAngle);
            auto errMeanAngle = m_ErrAngleRec->GetMean();
            angle += errMeanAngle * m_errAngleCoeff;
        }

        this->SetPosition(position, NWU);
        SetRotation( GetRotation().RotZ_RADIANS(angle, NWU));

        m_prevTime = time;
    }

}