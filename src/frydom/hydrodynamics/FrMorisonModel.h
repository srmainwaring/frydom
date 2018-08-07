//
// Created by camille on 17/04/18.
//

#ifndef FRYDOM_FRMORISONMODEL_H
#define FRYDOM_FRMORISONMODEL_H

#include <vector>

#include "frydom/core/FrConstants.h"
#include "frydom/core/FrNode.h"
#include "frydom/environment/waves/FrWaveField.h"
#include "frydom/environment/waves/FrFlowSensor.h"

namespace frydom {

    class FrMorisonForce;
    class FrHydroBody;
    //class FrFlowSensor;

    // --------------------------------------------------------------------------
    // MORISON ELEMENT
    // --------------------------------------------------------------------------

    class FrMorisonModel {

    protected:
        std::shared_ptr<FrMorisonForce> m_force;    ///< morison force pointer
        bool is_rigid;                              ///< Flag to identify if the structure is moving

    public:
        /// Update the morison force
        virtual void UpdateState() = 0;

        /// Return the morison force pointer
        virtual FrMorisonForce* GetForce() const { return m_force.get(); }

        /// Return the force applied on the body
        virtual chrono::ChVector<double> GetBodyForce() const = 0;

        /// Return the moment applied on the body
        virtual chrono::ChVector<double> GetBodyTorque() const = 0;

        /// Add force the hydro body
        virtual void AddForce(FrHydroBody* body) = 0;

        /// Define the body to which the morison model is applied
        virtual void SetBody(FrHydroBody* body, bool add_force=false) = 0;

        /// Set the morison model rigid
        void SetRigid(const bool rigid=true) { is_rigid=rigid; }

        virtual void Initialize() = 0;

    };

    // ---------------------------------------------------------------------------
    // SINGLE ELEMENT
    // ---------------------------------------------------------------------------

    class FrSingleElement : public FrMorisonModel {

    private:
        std::shared_ptr<FrNode> m_nodeA;        ///< First node of the element
        std::shared_ptr<FrNode> m_nodeB;        ///< Second node of the element
        chrono::ChFrame<> m_frame;              ///< Frame linked to the morison element (in global coordinate system)
        double m_cd;                            ///< Drag coefficient
        double m_ca;                            ///< mass coefficient
        double m_cf;                            ///< friction coefficient
        double m_diameter;                      ///< diameter  of the morison element (m)
        double m_length;                        ///< Length of the morison element (m)
        double m_volume;                        ///< volume of the morison element
        std::unique_ptr<FrFlowSensor> m_flow;   ///< computation of the flow velocity and acceleration
        chrono::ChVector<> m_dir;               ///< unit vector in the axis of the element

    public:
        FrSingleElement();

        /// Constructor from node position and morison parameters
        FrSingleElement(std::shared_ptr<FrNode>& nodeA,
                        std::shared_ptr<FrNode>& nodeB,
                        double diameter,
                        double ca,
                        double cd,
                        double cf);

        FrSingleElement(chrono::ChVector<>& posA, chrono::ChVector<>& posB,
                        double diameter, double ca, double cd, double cf);

        /// Definition of nodes from reference
        void SetNodes(FrNode& nodeA, FrNode& nodeB);

        /// Pass shared pointer for nodes
        void SetNodes(std::shared_ptr<FrNode>& nodeA, std::shared_ptr<FrNode>& nodeB);

        /// Definition of the added mass coefficient (-)
        void SetAddedMass(const double ca) { m_ca = ca;}

        /// Return the added mass coefficient (-)
        double GetAddedMass() const { return m_ca; }

        /// Definition of the adimentional drag coefficient (-)
        void SetDrag(const double cd) { m_cd = cd;}

        /// Return the adimentional drag coefficient (-)
        double GetDrag() const { return m_cd; }

        /// Definition of the friction coefficient (-) (for tangential force component)
        void SetFriction(const double cf) { m_cf = cf; }

        /// Return the friction coefficient (-)
        double GetFriction() const { return m_cf; }

        /// Definition of the equivalent cylinder diameter (m)
        void SetDiameter(const double diameter) { m_diameter = diameter; }

        /// Return the equivalent cylinder diameter (m)
        double GetDiameter() const {return m_diameter; }

        /// Add the morison force to the offshore structure
        void AddForce(FrHydroBody* body) override;

        /// Define the body to which the morison model is applied
        void SetBody(FrHydroBody* body, bool add_force=false) override;

        /// Definition of the flow sensor
        void SetFlowSensor(FrHydroBody* body);

        /// Return the flow sensor
        FrFlowSensor* GetFlowSensor() { return m_flow.get(); }

        /// Return the force applied on the body
        chrono::ChVector<double> GetBodyForce() const override;

        /// Return the moment applied to the body
        chrono::ChVector<double> GetBodyTorque() const override;

        // UPDATE

        /// Update position of the element for not fixed structures
        void UpdateFrame();

        /// Update the morison model
        void UpdateState() override;

        /// Initialisation of the morison element
        void Initialize() override;

    private:
        /// Return the water density from environment
        inline double WaterDensity() const;


    };

    // -----------------------------------------------------------------------------
    // COMPOSITE ELEMENT
    // -----------------------------------------------------------------------------

    class FrCompositeElement : public FrMorisonModel {

    protected:
        std::vector<std::unique_ptr<FrMorisonModel>> m_morison;
        double m_cd = 1.;                            ///< Default value for drag coefficient
        double m_ca = 1.;                            ///< Default value for mass coefficient
        double m_cf = 0.;                            ///< Default value for friction coefficient
        double m_diameter = 0.;                      ///< Default value for diameter of the morison element (m)
        bool is_global_force = false;                ///< yes : use resultant force ; no : considerate each force separatly

    public:
        /// Default constructor
        FrCompositeElement();

        /// Define if the resulting force should be used instead of each force separatly
        void SetGlobalForce(const bool global_force) {
            is_global_force = global_force;
        }

        /// Return if the resultant force is used
        bool GetGlobalForce() const { return is_global_force; }

        /// Activation of the resulting force
        void ActivateGlobalForce() { is_global_force = true; }

        /// Add a new element to the morison model
        void AddElement(FrMorisonModel* element) {
            m_morison.push_back(std::unique_ptr<FrMorisonModel>(element));
        }

        /// Add a new element from position
        void AddElement(chrono::ChVector<> posA,
                        chrono::ChVector<> posB,
                        double diameter,
                        double ca,
                        double cd,
                        double cf);

        /// Add a new element from position using default morison property
        void AddElement(chrono::ChVector<> posA,
                        chrono::ChVector<> posB);

        /// Add a new element with node reference
        void AddElement(std::shared_ptr<FrNode>& nodeA,
                        std::shared_ptr<FrNode>& nodeB,
                        double diameter,
                        double ca,
                        double cd,
                        double cf);

        /// Add a new element with node reference using default morison property
        void AddElement(std::shared_ptr<FrNode>& nodeA,
                        std::shared_ptr<FrNode>& nodeB);

        void AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const double dL,
                        double diameter, double ca, double cd, double cf);

        /// Add new element with discretization (element size)
        void AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const double dL);

        void AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const int n,
                        double diameter, double ca, double cd, double cf);

        /// Add new element with discretization (number of element)
        void AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const int n);

        /// Define the default value for drag coefficient
        void SetDefaultDrag(double cd) { m_cd = cd; }

        /// Get the default value for drag coefficient
        double GetDefaultDrag() const { return m_cd; }

        /// Define the default value for friction coefficient
        void SetDefaultFriction(double cf) { m_cf = cf; }

        /// Get the default value for friction coefficient
        double GetDefaultFriction() const { return m_cf; }

        /// Define the default value for mass coefficient
        void SetDefaultMass(double ca) { m_ca = ca; }

        /// Return the default mass coefficient
        double GetDefaultMass() const { return m_ca; }

        /// Define the default diameter
        void SetDefaultDiameter(double diameter) { m_diameter = diameter; }

        /// Return the default value of the diameter
        double GetDefaultDiameter() const { return m_diameter; }

        /// Set hydro body recursively to the morison model
        void SetBody(FrHydroBody* body, bool add_force=false) override {
            for (auto& element: m_morison) {
                element->SetBody(body, false);
            }
            this->AddForce(body);
        }

        /// Add force recursively to the morison model
        void AddForce(FrHydroBody* body) override;

        /// Compute the resulting force on the body
        chrono::ChVector<double> GetBodyForce() const override {
            chrono::ChVector<double> force(0.);
            for (auto& element : m_morison) {
                force += element->GetBodyForce();
            }
            return force;
        }

        /// Compute the resulting moment to the body
        chrono::ChVector<double> GetBodyTorque() const override {
            chrono::ChVector<double> moment(0.);
            for (auto& element: m_morison) {
                    moment += element->GetBodyTorque();
            }
            return moment;
        }

        /// Update state of the morison element
        void UpdateState() override;

        /// Call initialisation of the element contained in composite element
        void Initialize() override;
    };

}

#endif //FRYDOM_FRMORISONMODEL_H