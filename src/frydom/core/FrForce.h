//
// Created by frongere on 08/06/17.
//

#ifndef FRYDOM_FRFORCE_H
#define FRYDOM_FRFORCE_H

#include "frydom/core/FrGeographic.h"
#include "chrono/physics/ChForce.h"
#include "hermes/hermes.h"

#include "FrVector.h"

#include "FrObject.h"

// Forward declaration
//namespace chrono {
//    class ChForce;
//
//    template <class Real=double>
//    class ChVector;
//}

namespace frydom {

    class FrForce :
            public chrono::ChForce,
            public FrObject
    {

    protected:
//        chrono::ChBody* Body;
//        chrono::ChVector<> force = chrono::VNULL;
        chrono::ChVector<> moment = chrono::VNULL;
        hermes::Message m_log;
        bool is_log = true;

        std::string m_logPrefix = "";           //TODO : à definir dans hermes

    public:

//        FrForce() : moment(chrono::VNULL) {};
//        FrForce() = default;


        /// Updates the time of the object and other stuff that are time-dependent
        /// into the object
        void UpdateTime(double mytime) {
            ChTime = mytime;

            // ... put time-domain stuff here
        }

        /// Update the force object.
        /// Must be implemented into the child classes.
        virtual void UpdateState() = 0;

        /// Get the force-torque applied to rigid, body as force vector.
        /// CAUTION !!!! : The force must be returned in the absolute coordinates while the torque must be
        /// expressed in body coordinates
        void GetBodyForceTorque(chrono::ChVector<>& body_force, chrono::ChVector<>& body_torque) const {
            body_force = force;
            body_torque = moment;
        }

        /// Return the moment of the force
        virtual chrono::ChVector<> GetTorque() const { return moment; }

        virtual void Initialize() override {
            if (is_log) {
                SetLog();
                InitializeLogs();
            }
        }

        virtual void StepFinalize() override {
            if (is_log) {
                UpdateLogs();
            }
        }

        /// Set the definition of the log message
        virtual void SetLog();

        virtual void SetLogNameAndDescription(std::string name = "Force_message",
                                         std::string description = "Message of the force") {
            m_log.SetNameAndDescription(name, description);
            is_log = true;
        }

        /// Deactivate the generation of log from the body
        virtual void DeactivateLog() { is_log = false; };

        /// Initialization of the log message
        virtual void InitializeLogs();

        /// Update of the log message
        virtual void UpdateLogs();

        hermes::Message* GetLog() { return &m_log; }

        /// Definition of the log messsage name
        virtual void SetLogPrefix(std::string prefix_name = "") { m_logPrefix = prefix_name; };

    };





























    /// REFACTORING ------------->>>>>>>>>>>>><


    class FrForce_;
    struct _FrForceBase : public chrono::ChForce {

        FrForce_* m_frydomForce;
        chrono::ChVector<double> m_torque; // Expressed in body coordinates at COG


        explicit _FrForceBase(FrForce_* force);

        void UpdateState() override;

        void GetBodyForceTorque(chrono::ChVector<double>& body_force, chrono::ChVector<double>& body_torque) const override;

        void GetAbsForceNWU(Force &body_force) const;

        void GetLocalTorqueNWU(Moment &body_torque) const;

        void SetAbsForceNWU(const Force &body_force);

        void SetLocalTorqueNWU(const Moment &body_torque);


        friend class FrForce_;

    };


    // Forward declaration;
    class FrOffshoreSystem_;
    class FrBody_;
    class FrNode_;


    class FrForce_ : public FrObject {

    protected:

        FrBody_* m_body;

        std::shared_ptr<_FrForceBase> m_chronoForce;

        // Limits on forces to stabilize simulation
        bool m_limitForce = false;
        double m_forceLimit  = 1e20;  // Taking very high values by default in case we just set limit to true without
        double m_torqueLimit = 1e20;  // setting the values individually.


    public:

        FrForce_();

//        explicit FrForce_(FrBody_* body);

        virtual void Update(double time) = 0;

        FrOffshoreSystem_* GetSystem();



        // Force Limits

        void SetMaxForceLimit(double fmax);

        double GetMaxForceLimit() const;

        void SetMaxTorqueLimit(double tmax);

        double GetMaxTorqueLimit() const;

        void SetLimit(bool val);

        bool GetLimit() const;


        // Force Getters

        void GetAbsForce(Force& force, FRAME_CONVENTION fc) const;

        Force GetAbsForce(FRAME_CONVENTION fc) const;

        void GetAbsForce(double& fx, double& fy, double& fz, FRAME_CONVENTION fc) const;

        void GetLocalForce(Force& force, FRAME_CONVENTION fc) const;

        Force GetLocalForce(FRAME_CONVENTION fc) const;

        void GetLocalForce(double& fx, double& fy, double& fz, FRAME_CONVENTION fc) const;

        void GetAbsTorqueAtCOG(Moment &torque, FRAME_CONVENTION fc) const;

        Moment GetAbsTorqueAtCOG(FRAME_CONVENTION fc) const;

        void GetAbsTorqueAtCOG(double &mx, double &my, double &mz, FRAME_CONVENTION fc) const;

        void GetLocalTorqueAtCOG(Moment &torque, FRAME_CONVENTION fc) const;

        Moment GetLocalTorqueAtCOG(FRAME_CONVENTION fc) const;

        void GetLocalTorqueAtCOG(double &mx, double &my, double &mz, FRAME_CONVENTION fc) const;

        double GetForceNorm() const;

        double GetTorqueNormAtCOG() const;




    protected:

        std::shared_ptr<chrono::ChForce> GetChronoForce();

        friend class FrBody_;

        // The following methods are to be used in the implementation of Update method to set the force and torque
        // of the force model used

        /// Set the force expressed in absolute coordinates. It does not generate a torque
        void SetAbsForce(const Force& force, FRAME_CONVENTION fc);

        /// Set the force expressed in absolute coordinates applying to a point expressed in body coordinates.
        /// It generates a torque.
        void SetAbsForceOnLocalPoint(const Force& force, const Position& relPos, FRAME_CONVENTION fc);

        /// Set the force expressed in absolute coordinates applying to a point expressed in absolute coordinates.
        /// It generates a torque.
        void SetAbsForceOnAbsPoint(const Force& force, const Position& absPos, FRAME_CONVENTION fc);

        /// Set the force expressed in body coordinates. It does not generate a torque.
        void SetLocalForce(const Force& force, FRAME_CONVENTION fc);

        /// Set the force expressed in body coordinates applying to a point expressed in body coordinates.
        /// It generates a torque.
        void SetLocalForceOnLocalPoint(const Force& force, const Position& relPos, FRAME_CONVENTION fc);

        /// Set the force expressed in body coordinates applying to a point expressed in absolute coordinates.
        /// It generates a torque.
        void SetLocalForceOnAbsPoint(const Force& force, const Position& absPos, FRAME_CONVENTION fc);

        /// Set the torque expressed in absolute coordinates and at COG.
        void SetAbsTorqueAtCOG(const Moment& torque, FRAME_CONVENTION fc);

        /// Set the torque expressed in relative coordinates and at COG.
        void SetLocalTorqueAtCOG(const Moment& torque, FRAME_CONVENTION fc);

        /// Set force and torque expressed in absolute coordinates and at COG.
        void SetAbsForceTorqueAtCOG(const Force& force, const Moment& torque, FRAME_CONVENTION fc);

        /// Set force and torque expressed in body coordinates and at COG
        void SetLocalForceTorqueAtCOG(const Force& force, const Moment& torque, FRAME_CONVENTION fc);

        /// Set force and torque expressed in absolute coordinates and reduced to a point expressed in body coordinates
        void SetAbsForceTorqueAtLocalPoint(const Force& force, const Moment& torque, const Position& relPos, FRAME_CONVENTION fc);

        /// Set force and torque expressed in absolute coordinates and reduced to a point expressed in absolute coordinates
        void SetAbsForceTorqueAtAbsPoint(const Force& force, const Moment& torque, const Position& absPos, FRAME_CONVENTION fc);

        /// Set force and torque expressed in body coordinates and reduced to a point expressed in body coordinates
        void SetLocalForceTorqueAtLocalPoint(const Force& force, const Moment& torque, const Position& relPos, FRAME_CONVENTION fc);

        /// Set force and torque expressed in body coordinates and reduced to a point expressed in absolute coordinates
        void SetLocalForceTorqueAtAbsPoint(const Force& force, const Moment& torque, const Position& absPos, FRAME_CONVENTION fc);

    };











}  // end namespace frydom

#endif //FRYDOM_FRFORCE_H
