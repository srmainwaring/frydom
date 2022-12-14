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
                        Matrix33 &mat);

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

    ~FrCatenaryLineSeabed();

    void AddClumpWeight(double s, const double &mass, bool reversed = false);

    void AddBuoy(double s, const double &mass, bool reversed = false);

    void Initialize() override;

    Force GetTension(const double &s, FRAME_CONVENTION fc) const override;

    Direction GetTangent(const double s, FRAME_CONVENTION fc) const override;

    Force GetTensionAtTouchDown(FRAME_CONVENTION fc) const;

    Force GetTensionAtAnchor(FRAME_CONVENTION fc) const;

    bool HasTensionAtAnchor() const;

    Position GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const override;

    Position GetTouchDownPointPosition(FRAME_CONVENTION fc) const;

    double GetUnstretchedLength() const override;

    double GetUnstretchedLengthOnSeabed() const;

    void solve() override;

    bool HasSeabedInteraction() const override;

    void GetLowestPoint(Position &position,
                        double &s,
                        FRAME_CONVENTION fc,
                        const double tol,
                        const unsigned int maxIter) const override;

   private:

    using Residue4 = Eigen::Vector4d;
    using Jacobian44 = Eigen::Matrix4d;

    void SetLb(const double &Lb);

    void FirstGuess() override;

    int SolveTDPPosition();

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

}  // end namespace frydom

#endif //FRYDOM_FRCATENARYLINESEABED__H
