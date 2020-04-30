//
// Created by lletourn on 05/03/19.
//

#ifndef FRYDOM_FRFEACABLE_H
#define FRYDOM_FRFEACABLE_H

#include <chrono/fea/ChMesh.h>
//#include <chrono/fea/ChElementBeamEuler.h>
//#include <chrono/fea/ChElementBeamANCF.h>
//#include <chrono/fea/ChElementBeamIGA.h>
//#include <chrono/fea/ChElementCableANCF.h>
//#include <chrono/fea/ChBuilderBeam.h>
//#include <chrono/fea/ChBeamSectionCosserat.h>



#include "frydom/cable/common/FrCableBase.h"

#include "frydom/core/common/FrConvention.h"
#include "frydom/core/common/FrFEAMesh.h"
#include "frydom/core/math/FrVector.h"


#include "frydom/cable/fea/FrFEANode.h"


// Forward declaration
namespace chrono {
  namespace fea {
    class ChBeamSectionCosserat;
  }
  class ChLinkMateGeneric;
}


namespace frydom {

  namespace internal {

    class FrFEACableBase : public FrFEAMeshBase {

     public:
      explicit FrFEACableBase(FrFEACable *cable);

      std::shared_ptr<chrono::ChLinkMateGeneric> GetStartingHinge();

      std::shared_ptr<chrono::ChLinkMateGeneric> GetEndingHinge();

      // Overrides that are necessary to apply added mass effects on the cable

      void Initialize() override;

     private:

      void BuildProperties();

      void InitializeShape();

      void InitializeLoads();

      void InitializeLinks();

      void InitializeContacts();

      void InitializeAssets();

     private:
      FrFEACable* GetFEACable();

     private:

      // TODO: voir si on a besoin de cela...
      std::shared_ptr<FrFEANodeBase> m_starting_node_fea;  ///< Starting node
      std::shared_ptr<FrFEANodeBase> m_ending_node_fea;    ///< Ending node

      std::shared_ptr<chrono::ChLinkMateGeneric> m_starting_hinge;         ///< Starting hinge, to connect to a body
      std::shared_ptr<chrono::ChLinkMateGeneric> m_ending_hinge;           ///< Ending hinge, to connect to a body

      std::shared_ptr<chrono::fea::ChBeamSectionCosserat> m_section;

      unsigned int m_bspline_order; // TODO: rendre la chose parametrable (voir a le mettre dans FrFEACable)

    };

  }  // end namespace frydom::internal


  class FrFEACable : public FrCableBase, public FrFEAMesh {

   public:

    FrFEACable(const std::string &name,
               const std::shared_ptr<FrNode> &startingNode,
               const std::shared_ptr<FrNode> &endingNode,
               const std::shared_ptr<FrCableProperties> &properties,
               double unstretched_length,
               unsigned int nb_nodes);

    void FreeStarNode(bool val);

    void FreeEndNode(bool val);


    Force GetTension(const double &s, FRAME_CONVENTION fc) const override;

    Position GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const override;


    void Initialize() override;

    void StepFinalize() override;


    void Relax() override;

    double GetStaticResidual() override;

    unsigned int GetNbNodes() const;


   protected:

    void DefineLogMessages() override;

    void BuildCache() override;

//    std::shared_ptr<internal::FrFEACableBase> GetChronoCable() {
//      return std::dynamic_pointer_cast<internal::FrFEACableBase>(m_chrono_mesh);
//    }

    // friends
    friend bool FrOffshoreSystem::Add(std::shared_ptr<FrTreeNodeBase>);

   private:

    unsigned int m_nb_nodes;

//    bool m_is_start_node_free;
//    bool m_is_end_node_free;

  };


  std::shared_ptr<FrFEACable>
  make_fea_cable(const std::string &name,
                 const std::shared_ptr<FrNode> &startingNode,
                 const std::shared_ptr<FrNode> &endingNode,
                 const std::shared_ptr<FrCableProperties> &properties,
                 double unstretched_length,
                 unsigned int nb_elements);





















  //////////////>>>>>>>>>>>>>>>><   OLD CODE


//
//  // Forward declaration
//  class FrFEACable;
//
//  class FrCableShapeInitializer;
//
//  namespace internal {
//
//    class FrElementBeamEuler : public chrono::fea::ChElementBeamEuler {
//
//    };
//
//    class FrElementBeamANCF : public chrono::fea::ChElementBeamANCF {
//    };
//
//    class FrElementBeamIGA : public chrono::fea::ChElementBeamIGA {
//
//     public:
//
//      /// Gets the absolute xyz velocity of a point on the beam line, at abscissa 'eta'.
//      /// Note, eta=-1 at node1, eta=+1 at node2.
//      virtual void EvaluateSectionSpeed(const double eta,
//                                        chrono::ChVector<> &point_speed) {
//
//        // compute parameter in knot space from eta-1..+1
//        double u1 = knots(order); // extreme of span
//        double u2 = knots(knots.GetRows() - order - 1);
//        double u = u1 + ((eta + 1) / 2.0) * (u2 - u1);
//        int nspan = order;
//
//        chrono::ChVectorDynamic<> N((int) nodes.size());
//
//        chrono::geometry::ChBasisToolsBspline::BasisEvaluate(
//            this->order,
//            nspan,
//            u,
//            knots,
//            N);           ///< here return  in N
//
//        point_speed = chrono::VNULL;
//        for (int i = 0; i < nodes.size(); ++i) {
//          point_speed += N(i) * nodes[i]->coord_dt.pos;
//        }
//      }
//
//      /// Gets the absolute xyz position of a point on the beam line, at abscissa 'eta'.
//      /// Note, eta=-1 at node1, eta=+1 at node2.
//      virtual void EvaluateSectionAcceleration(const double eta,
//                                               chrono::ChVector<> &point_acceleration) {
//
//        // compute parameter in knot space from eta-1..+1
//        double u1 = knots(order); // extreme of span
//        double u2 = knots(knots.GetRows() - order - 1);
//        double u = u1 + ((eta + 1) / 2.0) * (u2 - u1);
//        int nspan = order;
//
//        chrono::ChVectorDynamic<> N((int) nodes.size());
//
//        chrono::geometry::ChBasisToolsBspline::BasisEvaluate(
//            this->order,
//            nspan,
//            u,
//            knots,
//            N);           ///< here return  in N
//
//        point_acceleration = chrono::VNULL;
//        for (int i = 0; i < nodes.size(); ++i) {
//          point_acceleration += N(i) * nodes[i]->coord_dtdt.pos;
//        }
//      }
//
//
//    };
//
//    class FrElementCableANCF : public chrono::fea::ChElementCableANCF {
//    };
//
//
//    /**
//     * /class FrFEACableBase
//     * /Brief Base class for the Dynamic Cable
//     * This class contains the Finite Element Analysis (FEA) mesh, starting and ending nodes and hinges, along with
//     * the section properties (linear density, section, inertia, Young modulus, Rayleigh damping, etc.)
//     *
//     * /see FrFEACable
//     */
//    struct FrFEACableBase : public chrono::fea::ChMesh {
//
//      FrFEACable *m_frydomCable;      ///< pointer to the Dynamic cable containing this base class
//
//      bool m_drawCableElements = true;    ///< Boolean to check if the FEA elements are to be drawnn
//      bool m_drawCableNodes = true;       ///< Boolean to check if the FEA nodes are to be drawnn
//
//      std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> m_starting_node_fea;  ///< Starting node
//      std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> m_ending_node_fea;    ///< Ending node
//
//      std::shared_ptr<chrono::ChLinkMateGeneric> m_starting_hinge;         ///< Starting hinge, to connect to a body
//      std::shared_ptr<chrono::ChLinkMateGeneric> m_ending_hinge;           ///< Ending hinge, to connect to a body
//
//
//      /// Constructor of the FrFEACableBase
//      /// \param cable pointer to the FrFEACable containing this base class
//      explicit FrFEACableBase(FrFEACable *cable);
//
//      /// Initialize the cable
//      void Initialize();
//
//      /// Update time dependent data, for all elements.
//      /// Updates all [A] coord.systems for all (corotational) elements.
//      void Update(double time, bool update_assets) override;
//
//      /// Get the position of the dynamic cable at the local position eta in [-1,1], of the indexth element
//      /// \param index index of the cable element
//      /// \param eta Local position on the element, in [-1,1]
//      /// \return Position of the cable, in the world reference frame
//      Position GetNodePositionInWorld(int index, double eta); // FIXME: le nom de cette methode n'est pas correct !
//
//      /// Get the axial tension (compression only, no bending) of the dynamic cable at the local position
//      /// eta in [-1,1], of the indexth element
//      /// \param index index of the cable element
//      /// \param eta Local position on the element, in [-1,1]
//      /// \return Axial tension of the dynamic cable, in world reference frame
//      Force GetTension(int index, double eta);
//
//      /// Initialize the links (hinges) between the cable and the bodies
//      //FYI : Can't be changed to private, since it's friend with FrBody (need chronoBody)
//      void InitializeLinks();
//
//     private:
//
////      /// Initialize the cable section
////      void InitializeSection();
//
//      void InitializeContact();
//
//      void InitializeHydrodynamicLoads();
//
//      /// Generate assets for the cable
//      void GenerateAssets();
//
//      /// Define the constraints in the hinges
//      void SetHingeConstraints();
//
//    };
//
//
////    class CableBuilderIGA : public chrono::fea::ChBuilderBeamIGA {
////
////     public:
////
////      CableBuilderIGA(FrFEACableBase *cable,
////                      FrCableProperties *properties,
////                      FrCableShapeInitializer *shape_initializer);
////
////      void Build();
////
////      std::shared_ptr<chrono::fea::ChBeamSectionCosserat> BuildSection(FrCableProperties *properties);
////
////      std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> GetFirstNode();
////
////      std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> GetLastNode();
////
////     private:
////      FrFEACableBase *m_cable;
////      FrCableProperties *m_properties;
////      FrCableShapeInitializer *m_shape_initializer;
////
////    };
//
//
//  }  // end namespace frydom::internal
//
//  /**
//   * \class FrFEACable FrFEACable.h
//   * \brief Class for dynamic cable, subclass of FrCableBase and FrFEAMesh
//   * The dynamic cable is based on a Finite Element Analysis (FEA) cable, with an Euler-Bernoulli formulation on a
//   * simple beam element with two nodes. The section and material properties are assumed constant along the beam.
//   *
//   *For more information, refer to : http://www.projectchrono.org/assets/white_papers/FEA/euler_beams.pdf
//   */
//  //TODO : Additional linear loads (Morison, hydrostatic, etc.)
//  //TODO : Breaking of cable
//  //TODO : Unrolling
//  //TODO : Contact with seabed or other cable/bodies
//  //TODO : Check for deactivation
//  class FrFEACable : public FrLoggable<FrOffshoreSystem>, public FrCableBase, public FrFEAMesh {
//   public:
//
//    enum BOUNDARY_CONSTRAINT_TYPE { // FIXME: en realite, on va a priori toujours considerer que c'est spherique !!!
//      CONSTRAINED,
//      SPHERICAL,
//      NONE
//    };
//
//   public:
//
//    /// Constructor of the Dynamic Cable
//    /// \param startingNode Starting node
//    /// \param endingNode Ending node
//    /// \param properties cable properties
//    /// \param unstrainedLength unstretched length of the cable, in m
//    /// \param rayleighDamping Rayleigh damping
//    /// \param nbElements Number of elements/discretization
//    FrFEACable(const std::string &name,
//               const std::shared_ptr<FrNode> &startingNode,
//               const std::shared_ptr<FrNode> &endingNode,
//               const std::shared_ptr<FrCableProperties> &properties,
//               double unstrainedLength,
//               double rayleighDamping,
//               unsigned int nbElements);
//
//    /// Get the FrOffshoreSystem
//    inline FrOffshoreSystem *GetSystem() const {
//      return FrTreeNode<FrOffshoreSystem>::GetParent();
//    }
//
//    /// Set the Rayleigh damping coefficient (only stiffness related coefficient is included in chrono model, as in: R = r * K )
//    /// \param damping Raleigh damping
//    void SetRayleighDamping(double damping);
//
//    /// Get the Rayleigh damping coefficient
//    /// \return Raleigh damping
//    double GetRayleighDamping() const;
//
//    /// Set the number of elements/discretization of the cable
//    /// \param nbElements Number of elements
//    void SetNumberOfElements(unsigned int nbElements);
//
//    /// Get the number of elements of the cable
//    /// \return Number of elements
//    unsigned int GetNumberOfElements() const;
//
//    /// Set the number of elements based on a target element length
//    /// \param ElementLength Target length of the elements
//    void SetTargetElementLength(double ElementLength);
//
//    //--------------------------------------------------------------------------------------------------------------
//    //Asset
//
//    /// Set the breaking tension of the cable (for visualization purpose only for now)
//    /// \param tension breaking tension
//    void SetBreakingTension(double tension);
//
//    /// Get the breaking tension of the cable
//    /// \return breaking tension
//    double GetBreakingTension() const;
//
//    /// Set the radius of the cable elements assets
//    /// \param radius Radius of the cable elements assets
//    void SetDrawElementRadius(double radius);
//
//    /// Get the radius of the cable elements assets
//    /// \return Radius of the cable elements assets
//    double GetDrawElementRadius();
//
//    /// Set the size of the cable nodes assets
//    /// \param size Size of the cable nodes assets
//    void SetDrawNodeSize(double size);
//
//    /// Get the size of the cable nodes assets
//    /// \return Size of the cable nodes assets
//    double GetDrawNodeSize() const;
//
//    //--------------------------------------------------------------------------------------------------------------
//    // Hinges
//    /// Set the starting hinge type (CONSTRAINED, SPHERICAL, NONE)
//    /// \param type starting hinge type (CONSTRAINED, SPHERICAL, NONE)
//    void SetStartingHingeType(BOUNDARY_CONSTRAINT_TYPE type);
//
//    /// Get the starting hinge type (CONSTRAINED, SPHERICAL, NONE)
//    /// \return starting hinge type (CONSTRAINED, SPHERICAL, NONE)
//    BOUNDARY_CONSTRAINT_TYPE GetStartingHingeType() const;
//
//    /// Set the ending hinge type (CONSTRAINED, SPHERICAL, NONE)
//    /// \param type ending hinge type (CONSTRAINED, SPHERICAL, NONE)
//    void SetEndingHingeType(BOUNDARY_CONSTRAINT_TYPE type);
//
//    /// Get the ending hinge type (CONSTRAINED, SPHERICAL, NONE)
//    /// \return ending hinge type (CONSTRAINED, SPHERICAL, NONE)
//    BOUNDARY_CONSTRAINT_TYPE GetEndingHingeType() const;
//
//    //--------------------------------------------------------------------------------------------------------------
//    // Virtual methods, from FrCableBase
//
//    /// Get the inside line tension at the lagrangian coordinate s, from the starting node to the ending node
//    /// \param s lagrangian coordinate
//    /// \param fc frame convention (NED/NWU)
//    /// \return inside line tension
//    Force GetTension(const double &s, FRAME_CONVENTION fc) const override;
//
//    Direction GetTangent(const double &s, FRAME_CONVENTION fc) const;
//
//    /// Get the line position at lagrangian coordinate s
//    /// \param s lagrangian coordinate
//    /// \param fc frame convention (NED/NWU)
//    /// \return line position
//    Position
//    GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const override; // FIXME: verifier que ca fonctionne !!
//
//    // Virtual methods, from FEAMesh
//
//    /// Initialize the cable with given data
//    void Initialize() override;
//
//    /// Update the internal parameters of the cable
//    /// \param time time of the simulation
//    void Update(double time) override {};
//
////      /// Initialize the log for the dynamic cable
////      void AddFields() override;
//
//    double GetStaticResidual() override;
//
//    void Relax() override;
//
//    std::shared_ptr<chrono::fea::ChBeamSectionCosserat> GetSection() const {
//      return m_section;
//    }
//
//   protected:
//    void BuildCache() override {}
//
//    void BuildSection();
//
//    void InitializeTaut();
//
//    void InitializeSlack();
//
//
//    // Friend definitions
//
//    friend bool FrOffshoreSystem::Add(std::shared_ptr<FrTreeNodeBase> item);
//
//    friend void FrOffshoreSystem::Remove(std::shared_ptr<FrTreeNodeBase> item);
//
//
//   private:
//
//    std::shared_ptr<internal::FrFEACableBase> m_chronoCable;    ///< pointer to the Chrono cable
//
//    double m_rayleighDamping;               ///< Rayleigh damping // FIXME: c'est pas deja dans les proprietes ???
//    unsigned int m_nbElements;              ///< Number of elements in the finite element cable model
//
//    std::shared_ptr<chrono::fea::ChElasticityCosseratSimple> m_elasticity;
//    std::shared_ptr<chrono::fea::ChBeamSectionCosserat> m_section;      ///< Section properties (linear density, section, inertia, Young modulus, Rayleigh damping, etc.)
//
//    // Hinges types
//    BOUNDARY_CONSTRAINT_TYPE m_startingHingeType = SPHERICAL; // TODO: retirer cette notion ????
//    BOUNDARY_CONSTRAINT_TYPE m_endingHingeType = SPHERICAL;
//
//    // Asset parameters
//    double m_drawCableElementRadius = 0.05; ///< Radius of the cable element assets
//    double m_drawCableNodeSize = 0.1;       ///< Size of the cable node assets
//    double m_maxTension = 0.;               ///< max tension, for visualization
//
//   protected:
//
//    void DefineLogMessages() override;
//
//    std::shared_ptr<chrono::fea::ChMesh> GetChronoMesh() override { return m_chronoCable; }
//
//  };
//
//  std::shared_ptr<FrFEACable>
//  make_dynamic_cable(const std::string &name,
//                     const std::shared_ptr<FrNode> &startingNode,
//                     const std::shared_ptr<FrNode> &endingNode,
//                     const std::shared_ptr<FrCableProperties> &properties,
//                     double unstrainedLength,
//                     double rayleighDamping,
//                     unsigned int nbElements);

} // end namespace frydom


#endif //FRYDOM_FRFEACABLE_H
