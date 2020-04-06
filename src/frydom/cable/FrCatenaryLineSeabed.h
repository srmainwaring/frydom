//
// Created by frongere on 27/02/2020.
//

#ifndef FRYDOM_FRCATENARYLINESEABED__H
#define FRYDOM_FRCATENARYLINESEABED__H


#include "FrCatenaryLineBase.h"


namespace frydom {

  // Forward declaration
  class FrCatenaryLine;

  class FrCatenaryLineSeabed;

  namespace internal {

    struct JacobianBuilder {
     public:

      static void dp_dt(const FrCatenaryLineSeabed &sl,
                        const FrCatenaryLine &cl,
                        mathutils::Matrix33<double> &mat);

      static void dp_dLb(const FrCatenaryLineSeabed &sl,
                         const FrCatenaryLine &cl,
                         mathutils::Vector3d<double> &vec);

      static void catenary_residue(const FrCatenaryLineSeabed &sl,
                                   const FrCatenaryLine &cl,
                                   mathutils::Vector3d<double> &vec);

      static void Lb_residue(const FrCatenaryLineSeabed &sl, double &scalar);

    };

  }  // end namespace frydom::internal


  class FrCatenaryLineSeabed : public FrCatenaryLineBase {

   public:
    FrCatenaryLineSeabed(const std::string &name,
                         const std::shared_ptr<FrNode> &anchorNode,
                         const std::shared_ptr<FrNode> &fairleadNode,
                         const std::shared_ptr<FrCableProperties> &properties,
                         bool elastic,
                         double unstretchedLength,
                         FLUID_TYPE fluid,
                         double seabed_friction_coeff);

    void AddClumpWeight(double s, const double &mass, bool reversed = false);

    void AddBuoy(double s, const double &mass, bool reversed = false);

    void Initialize() override;

    Force GetTension(const double &s, FRAME_CONVENTION fc) const override;

    Force GetTensionAtTouchDown(FRAME_CONVENTION fc) const;

    Force GetTensionAtAnchor(FRAME_CONVENTION fc) const;

    bool HasTensionAtAnchor() const;

    Position GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const override;

    Position GetTouchDownPointPosition(FRAME_CONVENTION fc) const;

    double GetUnstretchedLength() const override;

    double GetUnstretchedLengthOnSeabed() const;

    void solve() override;

   private:

    using Residue4 = Eigen::Vector4d;
    using Jacobian44 = Eigen::Matrix4d;

    void SetLb(const double &Lb);

    void FirstGuess() override;

    void BuildCache() override;

    double gamma() const;

    Position p(const double &s) const;

    Position p_seabed(const double &s) const;

    Position p_catenary(const double &s) const;

    Tension t(const double &s) const;

    Tension t_seabed(const double &s) const;

    Tension t_catenary(const double &s) const;

    Tension t_TDP() const;

    void GetJacobian(Jacobian44 &jacobian) const;

    void GetResidue(Residue4 &residue) const;

    Direction GetCatenaryPlaneIntersectionWithSeabed(FRAME_CONVENTION fc) const;

    void Compute(double time) override;

    internal::FrPhysicsItemBase *GetChronoItem_ptr() const override; // Qu'est ce que ca fait la ???

    void DefineLogMessages() override;

    friend class internal::JacobianBuilder;

   private:

    std::unique_ptr<FrCatenaryLine> m_catenary_line;
    std::shared_ptr<FrNode> m_touch_down_node;

    double m_Cb;

    double m_Lb;

    double c_qL;


  };

  std::shared_ptr<FrCatenaryLineSeabed>
  make_catenary_line_seabed(const std::string &name,
                            const std::shared_ptr<FrNode> &startingNode,
                            const std::shared_ptr<FrNode> &endingNode,
                            const std::shared_ptr<FrCableProperties> &properties,
                            bool elastic,
                            double unstretchedLength,
                            FLUID_TYPE fluid_type,
                            double seabed_friction_coeff);




//
//    void Initialize() override;
//
//    void solve() override;
//
////    Force GetTensionAtTouchDownPoint() const {
////      return m_t0;
////    }
//
//    Position GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const override;
//
//    Force GetTension(const double &s, FRAME_CONVENTION fc) const override;
//
//    double GetUnstretchedLength() const override;
//
//   private:
//
//    FrNode *GetAnchorNode() {
//      return m_anchor_node.get();
//    }
//
//    FrNode *GetFairleadNode() {
//      return m_endingNode.get();
//    }
//
//    FrNode *GetTouchDownPointNode() {
//      return m_startingNode.get();
//    }
//
//    inline double GetUnstretchedLengthCatenaryPart() const;
//
//    void CorrectTouchDownPointAbscissae(const double &correction);
//
//    void UpdateTouchDownPointPosition() {
//      m_startingNode->SetPositionInWorld(m_anchor_node->GetPositionInWorld(NWU)
//                                         + m_Lb * m_lying_direction,
//                                         NWU);
//    }
//
//    double GetSeabedIntersection() const;
//
//    Position GetPositionOnSeabed(double s, FRAME_CONVENTION fc) const;
//
//    mathutils::VectorN<double> get_residual_seabed() const;
//
//    mathutils::MatrixMN<double> analytical_jacobian_seabed() const;
//
//    inline double GetHorizontalTensionAtTouchDownPoint() const;
//
//   private:
//
//    std::shared_ptr<FrNode> m_anchor_node;
//
//    double m_Lb; // Abscisse lagrangienne du TDP
//    bool m_use_seabed_interaction_solver;
//
//    Direction m_lying_direction;
//
//    double c_Cb;
//
//  };
//
//  std::shared_ptr<FrCatenaryLineSeabed> make_catenary_line_seabed(const std::string &name,
//                                                                  const std::shared_ptr<FrNode> &anchorNode,
//                                                                  const std::shared_ptr<FrNode> &fairleadNode,
//                                                                  const std::shared_ptr<FrCableProperties> &properties,
//                                                                  bool elastic,
//                                                                  double unstretchedLength,
//                                                                  FLUID_TYPE fluid) {
//    auto line = std::make_shared<FrCatenaryLineSeabed>(name,
//                                                       anchorNode,
//                                                       fairleadNode,
//                                                       properties,
//                                                       elastic,
//                                                       unstretchedLength,
//                                                       fluid);
//    anchorNode->GetBody()->GetSystem()->Add(line);
//  }
//
//
}  // end namespace frydom

#endif //FRYDOM_FRCATENARYLINESEABED__H
