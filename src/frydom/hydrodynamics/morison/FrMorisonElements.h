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


#ifndef FRYDOM_FRMORISONELEMENTS_H
#define FRYDOM_FRMORISONELEMENTS_H

#include <memory>

#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrFrame.h"

#include "frydom/core/common/FrPhysicsItem.h"
#include "frydom/hydrodynamics/morison/FrMorisonModelBaseKRM.h"

namespace frydom {

  // Forward declarations
  class FrMorisonCompositeElement;

  class FrBody;

  class FrNode;

  // ---------------------------------------------------------
  // Makers
  // ---------------------------------------------------------

  /// Maker for a Morison model : instantiate and return a FrMorisonCompositeElement
  /// \param body body related to the morison model
  /// \return Morison model, as a Morison composite element
  std::shared_ptr<FrMorisonCompositeElement> make_morison_model(const std::string &name,
                                                                const std::shared_ptr<FrBody> &body,
                                                                bool extendedModel = false);

  /// Maker for a Morison model : instantiate and return a FrMorisonCompositeElement
  /// \param body body related to the morison model
  /// \return Morison model, as a Morison composite element
  std::shared_ptr<FrMorisonCompositeElement> make_morison_model(const std::string &name,
                                                                const std::shared_ptr<FrBody> &body,
                                                                const std::string &filename,
                                                                bool extendedModel = false);

  // ----------------------------------------------------------
  // Morison structure coefficient
  // ----------------------------------------------------------

  /// Morison coefficient structure used to allow isotropic or anisotropic coefficients definition
  /// for the morison model
  struct MorisonCoeff {
    double x;
    double y;
    double z;

    MorisonCoeff() {}

    MorisonCoeff(double val) {
      x = val;
      y = val;
      z = 0.;
    }

    MorisonCoeff(double val1, double val2) {
      x = val1;
      y = val2;
      z = 0.;
    }

    MorisonCoeff &operator=(double val) {
      x = val;
      y = val;
      z = 0.;
      return *this;
    }

    MorisonCoeff(double val1, double val2, double val3) {
      x = val1;
      y = val2;
      z = val3;
    }

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  /// The MorisonElementProperty structure encapsulate the basics property of a morison model
  struct MorisonElementProperty {
    MorisonCoeff cd = 0.;                   ///< Drag coefficient (can be isotropic or anisotropic)
    MorisonCoeff ca = 0.;                   ///< Added mass (can be isotropic ar anisotropic)
    double cf = 0.;                   ///< Friction coefficient
    double diameter = 0.;                   ///< Diameter of the morison model, in meter
    double length = 0.;                   ///< Length of the morison model, in meter
    double volume = 0.;                   ///< Volume of the morison model, in m³

    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // --------------------------------------------------------------------------
  // MORISON ELEMENT (base element)
  // --------------------------------------------------------------------------

  /// This class is a base class for morison model with only one single element or composite elements.
  class FrMorisonElement {

   protected:
    std::shared_ptr<FrNode> m_node;   ///< Frame at the center position of the morison element with z-axis along its direction
    Force m_force;                    ///< Force at COG of the body in world-coordinates
    Torque m_torque;                  ///< Torque at COG of the body in body-coordinates

    bool m_includeCurrent;            ///< Include current flow in morison element model

    bool m_simpleAMModel;             ///< If true, a simple model is used to compute the added mass components to be included the FrMorisonForce
    bool m_extendedModel;             ///< If true the inertial component of the morison force is used (false by dafault)
    Force m_force_added_mass;
    Torque m_torque_added_mass;

    bool m_isImmerged;

    Eigen::Matrix<double, 6, 6> m_AMInFrame; // FIXME : modifier type et nom
    Eigen::Matrix<double, 6, 6> m_AMInBody;
    Eigen::Matrix<double, 6, 6> m_AMInWorld;

   public:
    virtual ~FrMorisonElement();

    FrMorisonElement();

    /// Set the local frame of the morison model from node positions
    /// \param body Body to which the frame is attached
    /// \param posA Position of the first extremity of the morison element
    /// \param posB Position of the second extremity of the morison element
    /// \param vect x-axis of the frame is built such as is perpendicular to the morison element direction and this vector
    void SetFrame(FrBody *body, Position posA, Position posB, Direction vect = Direction(0., 0., 1.));

    /// Set the local frame of the morison model from another frame
    /// \param body Body to which the frame is attached
    /// \param frame Other frame
    void SetFrame(FrBody *body, const FrFrame &frame);

    /// Get the local frame of the morison model
    /// \return Local frame
    FrFrame GetFrame() const;

    /// Get the local frame of the morison model as node
    /// \return Node
    std::shared_ptr<FrNode> GetNode() const { return m_node; }

    FrBody *GetBody() const;

    /// Get the force vector at COG in world reference frame
    /// \param fc Frame convention
    /// \return Force vector
    Force GetForceInWorld(FRAME_CONVENTION fc) const;

    /// Get the torque vector at COG in body reference frame
    /// \return Torque vector
    Torque GetTorqueInBody() const;

    /// Get
    Force GetForceAddedMassInWorld(FRAME_CONVENTION fc) const;

    Torque GetTorqueAddedMassInWorld() const;

    /// Include current flow in the morison model
    /// \param includeCurrent Boolean, if true the current is included in morison model
    virtual void SetIncludeCurrent(bool includeCurrent) { m_includeCurrent = includeCurrent; }

    /// Defines if the inertial component with added mass is used (false by default)
    /// \param extendedModel Boolean, if true inertial component is used
    void SetExtendedModel(bool extendedModel) { m_extendedModel = extendedModel; }

    bool IsExtendedModel() const { return m_extendedModel; }

    void SetSimpleAMModel(bool simpleAMModel) { m_simpleAMModel = simpleAMModel; }

    bool IsSimpleAMModel() const { return m_simpleAMModel; }

    virtual const Eigen::Matrix<double, 6, 6> &GetAMInFrame();

    virtual const Eigen::Matrix<double, 6, 6> &GetAMInBody();

    virtual const Eigen::Matrix<double, 6, 6> &GetAMInWorld();

    bool IsImmerged() const { return m_isImmerged; }

    /// Update the force of the morison model
    /// \param time Current time of the simulation from begining (in seconds)
    virtual void Update(double time) = 0;

    /// Initialize the morison model
    virtual void Initialize() = 0;

    virtual void ComputeForceAddedMass() = 0;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };


  // --------------------------------------------------------------------------
  // MORISON SINGLE ELEMENT
  // --------------------------------------------------------------------------

  /// This class defines a morison model.
  /// It can be instanciate when the morison model is composed by only one single element
  /// The pointer to the body must be specified before to create a new morison model with single element
  class FrMorisonSingleElement : public FrMorisonElement {

   protected:
    std::shared_ptr<FrNode> m_nodeA;               ///< First extremity node of the morison element
    std::shared_ptr<FrNode> m_nodeB;               ///< Second extremity node of the morison element

    MorisonElementProperty m_property;              ///< Container of the morison property (diameter, drag coeff...)


   public:
    /// Constructor of a new morison element with property
    /// \param nodeA First extremity node of the morison element
    /// \param nodeB Second extremity node of the morison element
    /// \param diameter Diameter, in m
    /// \param ca Added mass coefficient
    /// \param cd Drag coefficient
    /// \param cf Friction coefficient
    /// \param perpendicular x-axis is built such as is perpendicular to the morison element direction and this vector
    FrMorisonSingleElement(std::shared_ptr<FrNode> &nodeA,
                           std::shared_ptr<FrNode> &nodeB,
                           double diameter, MorisonCoeff ca, MorisonCoeff cd, double cf,
                           const Direction &perpendicular = Direction(0., 0., 1.));

    /// Constructor of a new element with property
    /// \param body Body to which the morison element force is applied
    /// \param posA First extremity position of the morison element
    /// \param posB Second extremity position of the morison element
    /// \param diameter Diameter of the morison element
    /// \param ca Added mass
    /// \param cd Drag coefficient
    /// \param cf Friction coefficient
    /// \param perpendicular x-axis is built such as is perpendicular to the morison element direction and this vector
    FrMorisonSingleElement(FrBody *body, Position posA, Position posB,
                           double diameter, MorisonCoeff ca, MorisonCoeff cd, double cf,
                           const Direction &perpendicular = Direction(0., 0., 1.));

    //
    // Getters
    //

    /// Get the diameter of the morison element
    /// \return Diameter in meter
    double GetDiameter() const { return m_property.diameter; }

    /// Get the volume of the morison element
    /// \return Volume in m^3
    double GetVolume() const { return m_property.volume; }

    /// Get the length of the morison element
    /// \return length in meter
    double GetLength() const { return m_property.length; }

    //
    // UPDATE
    //

    /// Update the force of the morison element
    /// \param time Current time of the simulation from begining
    void Update(double time) override;

    /// Initialize the morison model
    void Initialize() override;

    void ComputeForceAddedMass() override;

   protected:

    /// Defines nodes from position and body link
    /// \param body Body to which the nodes are linked
    /// \param posA Position of the first node
    /// \param posB Position of the second node
    void SetNodes(FrBody *body, Position posA, Position posB);

    /// Set length of the morison element from node positions
    /// \param posA First extremity position of the element
    /// \param posB Second extremity position of the element
    void SetLength(Position posA, Position posB);

    /// Set morison element properties:
    ///     ca : added mass coefficient along the x-axis and y-axis in local frame
    ///     cd : drag coefficient along the x-axis and y-axis in local frame
    ///     cf : the friction coefficient along the z-axis (axial to the element)
    ///     diameter : diameter of the morison element
    void SetAddedMassCoeff(MorisonCoeff ca);

    void SetDragCoeff(MorisonCoeff cd);

    void SetFrictionCoeff(double cf);

    void SetDiameter(double diameter);

    /// Compute volume from the morison element diameter and length
    void SetVolume();

    /// Build the equivalent added mass matrix when full morison is used
    /// in local, body and world frames
    void SetAMInFrame();

    void SetAMInBody();

    void SetAMInWorld();

    /// Check if the morison element is under water or not
    void CheckImmersion();

    /// Get the relative flow velocity at frame position in local frame
    Velocity GetFlowVelocity();

    /// Get the relative flow acceleration at frame position in local frame
    Acceleration GetFlowAcceleration();

    /// Get the acceleration of the morison element in local frame
    Acceleration GetNodeAcceleration();

    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // --------------------------------------------------------------------------
  // MORISON COMPOSITE ELEMENT
  // --------------------------------------------------------------------------


  /// This class is used to build a composite morison model with multiple morison element.
  /// Composition of composite model can also be set up from this class.
  /// The resultant force and torque are the sum of the force and torque of each morison model component
  /// computed at the center of gravity of the body. The force is expressed in the world coordinates system
  /// and the torque in the body coordinate system.
  class FrMorisonCompositeElement : public FrMorisonElement,
                                    public FrPhysicsItem,
                                    public FrTreeNode<FrBody> {

   protected:
    std::vector<std::unique_ptr<FrMorisonElement>> m_morison;      ///< morison model components of the composite model

    std::shared_ptr<internal::FrMorisonModelBaseKRM> m_chronoAddedMass;

   public:
    /// Constructor of a new composite model of the morison force
    /// \param body Body to which the morison model is applied
    explicit FrMorisonCompositeElement(const std::string &name, FrBody *body, bool extendedModel = false);

    //TODO: remove unecessary AddElement methods, add if needed the frame convention for the position and complete the
    // doc (pos are in the world reference frame)

    /// Add a new morison model to the composite model
    /// \param model Morison model (can be single or composite elements)
    void AddElement(FrMorisonElement *model) {
      m_morison.push_back(std::unique_ptr<FrMorisonElement>(model));
    }

    /// Add a new single element to the composite model from nodes
    /// \param nodeA Node at the first extremity of the morison element
    /// \param nodeB Node at the second extremity of the morison element
    /// \param diameter Diameter of the morison element
    /// \param ca Added mass coefficient
    /// \param cd Drag coefficient
    /// \param cf Friction coefficient
    /// \param perpendicular x-axis is built such as is perpendicular to the morison element direction and this vector
    void AddElement(std::shared_ptr<FrNode> &nodeA, std::shared_ptr<FrNode> &nodeB,
                    double diameter, MorisonCoeff ca, MorisonCoeff cd, double cf,
                    Direction perpendicular = Direction(0., 0, 1.));

    /// Add a new single element to the composite model from positions
    /// \param posA Position of the first extremity of the morison element
    /// \param posB Position of the second extremity of the morison element
    /// \param diameter Diamter of the morison element
    /// \param ca Added mass coefficient
    /// \param cd Drag coefficient
    /// \param cf Friction coefficient
    /// \param n Number of discrete elements along the morison element
    /// \param perpendicular x-axis is built such as is perpendicular to the morison element direction and this vector
    void AddElement(Position posA, Position posB, double diameter,
                    MorisonCoeff ca, MorisonCoeff cd, double cf, unsigned int n = 1,
                    Direction perpendicular = Direction(0., 0., 1.));


    /// Include current flow in the morison model
    /// \param includeCurrent Boolean, if true the current is included in morison model
    void SetIncludeCurrent(bool includeCurrent) override {
      FrMorisonElement::SetIncludeCurrent(includeCurrent);
      for (auto& element:m_morison) element->SetIncludeCurrent(includeCurrent);
    }

    //
    // UPDATE
    //

    void Compute(double time) override;

    /// Update the force of the morison composite model
    /// \param time Current time of the simulation from begin
    void Update(double time) override;

    /// Initialize the morison composite model
    void Initialize() override;

    void StepFinalize() override {}

    // TODO : a voir pour externaliser dans une autre classe
    void ComputeForceAddedMass() override;

    protected:
      friend std::shared_ptr<internal::FrMorisonModelBaseKRM> internal::GetChronoMorisonAddedMass(std::shared_ptr<FrMorisonCompositeElement> morisonModel);

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

}  // end namespace frydom

#endif //FRYDOM_FRMORISONELEMENTS_H
