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


#ifndef FRYDOM_FRCABLEBASE_H
#define FRYDOM_FRCABLEBASE_H


#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrConvention.h"
#include "frydom/core/common/FrTreeNode.h"
#include "frydom/logging/FrLoggable.h"


namespace frydom {

  // Forward declaration
  class FrNode;

  class FrCableProperties;


  /**
   * \class FrCableBase FrCableBase.h
   * \brief Abstract base class for cables, superclass of FrCatenaryLine and
   * FrFEACable .
   * This means cables are updated between bodies and links.
   * A cable is connected to two nodes : a starting node and an ending node. Nodes are contained by at
   * least one body, and used to connect bodies to other components (cables, links,etc.)
   * \see FrCatenaryLine, FrFEACable, FrNode
   *
   */
  class FrCableBase {
    // TODO: voir pourquoi FrCableBase n'est pas un TreeNode<FrOffshoreSystem> ou un FrLoggable<FrOffshoreSystem>...

   protected:

    //--------------------------------------------------------------------------------------------------------------
    // time cached values
    double m_time = 0.;                         ///< cached value of the simulation time
    double m_time_step = 0.;                    ///< cached value of the simulation time step

    //--------------------------------------------------------------------------------------------------------------
    // Nodes
    std::shared_ptr<FrNode> m_startingNode;       ///< starting node
    std::shared_ptr<FrNode> m_endingNode;         ///< ending node

    //--------------------------------------------------------------------------------------------------------------
    // Cable properties
    std::shared_ptr<FrCableProperties> m_properties;    ///< Cable properties (section, Young modulus, linear density, etc.)

    double m_unstretchedLength;                    ///< Unstrained length of the cable in m
    double m_unrollingSpeed = 0;                        ///< linear unrolling speed of the cable in m/s
//        double m_breakingTension = 0;                       ///< breaking tension in N (for visualization purpose for now)

   public:

    /// FrCableBase constructor, using two nodes
    /// \param startingNode starting node
    /// \param endingNode ending node
    FrCableBase(const std::shared_ptr<FrNode> &startingNode,
                const std::shared_ptr<FrNode> &endingNode);

    /// FrCableBase constructor, using two nodes and cable properties
    /// \param startingNode starting node
    /// \param endingNode ending node
    /// \param properties cable properties
    /// \param unstretchedLength unstrained length, in m
    FrCableBase(const std::shared_ptr<FrNode> &startingNode,
                const std::shared_ptr<FrNode> &endingNode,
                const std::shared_ptr<FrCableProperties> &properties,
                double unstretchedLength);

    void Initialize();

    //--------------------------------------------------------------------------------------------------------------

    /// Set the cable properties (section, Young modulus, linear density, etc.)
    /// \param prop Cable properties
    void SetCableProperties(const std::shared_ptr<FrCableProperties> prop);

    /// Get the cable properties (section, Young modulus, linear density, etc.)
    /// \return prop Cable properties
    std::shared_ptr<FrCableProperties> GetProperties() const;

    /// Set the unstrained length of the cable
    /// \param L unstrained length
    void SetUnstretchedLength(double L);

    /// Get the unstrained length of the cable
    /// \return unstrained length
    virtual double GetUnstretchedLength() const;

    /// Set the linear unrolling speed of the cable in m/s
    void SetUnrollingSpeed(double unrollingSpeed);

    /// Get the linear unrolling speed of the cable in m/s
    double GetUnrollingSpeed() const;

    //--------------------------------------------------------------------------------------------------------------
    // Node accessors
    /// Set the starting node of the cable
    /// \param startingNode starting node
    void SetStartingNode(const std::shared_ptr<FrNode> startingNode);

    /// Get the starting node of the cable
    /// \return starting node
    std::shared_ptr<FrNode> GetStartingNode() const;

    /// Set the ending node of the cable
    /// \param endingNode ending node
    void SetEndingNode(const std::shared_ptr<FrNode> endingNode);

    /// Get the ending node of the cable
    /// \return ending node
    std::shared_ptr<FrNode> GetEndingNode() const;

    //--------------------------------------------------------------------------------------------------------------
    // pure virtual methods

    /// Get the inside line tension at the lagrangian coordinate s, from the starting node to the ending node
    /// \param s lagrangian coordinate
    /// \param fc frame convention (NED/NWU)
    /// \return inside line tension
    virtual Force GetTension(const double &s, FRAME_CONVENTION fc) const = 0;

    /// Get the line position at lagrangian coordinate s
    /// \param s lagrangian coordinate
    /// \param fc frame convention (NED/NWU)
    /// \return line position
    virtual Position GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const = 0;

    /// Get the strained length of the cable
    /// \return strained length
    virtual double GetStrainedLength() const;

//    protected:
//
//        virtual void InitBreakingTension();
//
//    public:
//
//        //--------------------------------------------------------------------------------------------------------------
//        /// Set the breaking tension of the cable (for visualization purpose only for now)
//        /// \param tension breaking tension
//        void SetBreakingTension(double tension);
//
//        /// Get the breaking tension of the cable
//        /// \return breaking tension
//        double GetBreakingTension() const;

    /// Update internal time and time step for dynamic behaviour of the cable
    /// \param time time of the simulation
    virtual void UpdateTime(double time);

    /// Update the length of the cable if unrolling speed is defined.
    virtual void UpdateState();


   protected:
    virtual void BuildCache() = 0;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };


}  // end namespace frydom


#endif //FRYDOM_FRCABLEBASE_H
