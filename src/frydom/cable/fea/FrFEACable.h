//
// Created by lletourn on 05/03/19.
//

#ifndef FRYDOM_FRFEACABLE_H
#define FRYDOM_FRFEACABLE_H

#include <chrono/fea/ChMesh.h>

#include "frydom/core/common/FrConvention.h"
#include "frydom/core/common/FrFEAMesh.h"
#include "frydom/core/math/FrVector.h"

#include "frydom/cable/common/FrCableBase.h"
#include "frydom/cable/fea/FrFEANode.h"


// Chrono Forward declarations
namespace chrono {
  namespace fea {
    class ChBeamSectionCosserat;
  }
  class ChLinkMateGeneric;
}


namespace frydom {


  // Forward declarations
  namespace internal {
    std::shared_ptr<internal::FrFEACableBase> GetChronoFEAMesh(std::shared_ptr<FrFEACable>);
  }

  class FrClumpWeight;


  class FrFEACable : public FrCableBase, public FrFEAMesh {

   public:
    enum FEA_BODY_CONSTRAINT_TYPE {
      FREE,
      SPHERICAL,
      FIXED
    };

   public:

    FrFEACable(const std::string &name,
               const std::shared_ptr<FrNode> &startingNode,
               const std::shared_ptr<FrNode> &endingNode,
               const std::shared_ptr<FrCableProperties> &properties,
               double unstretched_length,
               unsigned int nb_nodes);

    Force GetTension(const double &s, FRAME_CONVENTION fc) const override;

    //Force GetForceAtStartLink(FRAME_CONVENTION fc);

    //Force GetForceAtEndLink(FRAME_CONVENTION fc);

    Position GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const override;

    void Initialize() override;

    void StepFinalize() override;

    void Relax() override;

    double GetStaticResidual() override;

    unsigned int GetNbNodes() const;

    FEA_BODY_CONSTRAINT_TYPE GetStartLinkType() const;

    void SetStartLinkType(FEA_BODY_CONSTRAINT_TYPE ctype);

    FEA_BODY_CONSTRAINT_TYPE GetEndLinkType() const;

    void SetEndLinkType(FEA_BODY_CONSTRAINT_TYPE ctype);

    std::shared_ptr<FrClumpWeight> AddClumpWeight(const std::string &name, const double &s, const double &distance);

    Force GetForceStartNodeInWorld(FRAME_CONVENTION fc);

    Force GetForceEndNodeInWorld(FRAME_CONVENTION fc);

    Force GetForceEndNodeInBody(FRAME_CONVENTION fc);

   protected:

    void InitializeClumpWeights();

    std::shared_ptr<internal::FrFEANodeBase> GetNearestFEANode(const double &s);

    void DefineLogMessages() override;

    void BuildCache() override;

    internal::FrFEACableBase *GetFrFEACableBase(); // FIXME: bof le nom, on a aussi un GetFEAMeshBase dans FrFEAMesh...

    // friends
    friend std::shared_ptr<internal::FrFEACableBase> internal::GetChronoFEAMesh(std::shared_ptr<FrFEACable>);

   private:
    struct ClumpWeight_ {

      ClumpWeight_(std::shared_ptr<FrClumpWeight> clump_weight, const double &abscissa, const double &distance) :
          m_clump_weight(clump_weight),
          m_abscissa(abscissa),
          m_distance_to_node(distance) {}

      std::shared_ptr<FrClumpWeight> m_clump_weight;
      double m_abscissa;
      double m_distance_to_node;
    };


   private:

    unsigned int m_nb_nodes;

    FEA_BODY_CONSTRAINT_TYPE m_start_link_type;
    FEA_BODY_CONSTRAINT_TYPE m_end_link_type;

    std::vector<ClumpWeight_> m_clump_weights;

  };


  std::shared_ptr<FrFEACable>
  make_fea_cable(const std::string &name,
                 const std::shared_ptr<FrNode> &startingNode,
                 const std::shared_ptr<FrNode> &endingNode,
                 const std::shared_ptr<FrCableProperties> &properties,
                 double unstretched_length,
                 unsigned int nb_elements);


  namespace internal {

    class FrFEACableSection;

    class FrFEALinkBase;

    class FrFEACableBase : public FrFEAMeshBase {

     public:
      explicit FrFEACableBase(FrFEACable *cable);

      std::shared_ptr<FrFEALinkBase> GetStartLink();

      std::shared_ptr<FrFEALinkBase> GetEndLink();

      // Overrides that are necessary to apply added mass effects on the cable

      void Initialize() override;

      void SetupInitial() override;

      void SetStartLinkConstraint(FrFEACable::FEA_BODY_CONSTRAINT_TYPE ctype);

      void SetEndLinkConstraint(FrFEACable::FEA_BODY_CONSTRAINT_TYPE ctype);

      void BuildProperties();

      void InitializeShape();

      void InitializeLoads();

      void InitializeLinks();

      void InitializeContacts();

      void InitializeAssets();

      void SetLinkConstraint(FrFEACable::FEA_BODY_CONSTRAINT_TYPE start_ctype, FrFEALinkBase *link);

      FrFEACable *GetFEACable();

      std::shared_ptr<FrFEANodeBase> GetStartNodeFEA();

      std::shared_ptr<FrFEANodeBase> GetEndNodeFEA();

      std::shared_ptr<internal::FrFEANodeBase> GetNearestFEANode(const double &s);

     private:

      std::shared_ptr<FrFEALinkBase> m_start_link;         ///< Starting hinge, to connect to a body
      std::shared_ptr<FrFEALinkBase> m_end_link;           ///< Ending hinge, to connect to a body

      std::shared_ptr<FrFEACableSection> m_section;

      unsigned int m_bspline_order; // TODO: rendre la chose parametrable (voir a le mettre dans FrFEACable)

      std::vector<double> m_control_points_abscissa;

    };

    std::shared_ptr<FrFEACableBase> GetChronoFEAMesh(std::shared_ptr<FrFEACable> cable);

  }  // end namespace frydom::internal


} // end namespace frydom


#endif //FRYDOM_FRFEACABLE_H
