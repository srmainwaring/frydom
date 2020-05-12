//
// Created by frongere on 26/02/2020.
//

#ifndef FRYDOM_FRCLUMPWEIGHT_H
#define FRYDOM_FRCLUMPWEIGHT_H


#include <chrono/physics/ChLinkDistance.h>

#include "frydom/core/body/FrInertiaTensor.h"

#include "frydom/core/math/Fr3DGeometry.h"
#include "frydom/core/common/FrObject.h"
#include "frydom/logging/FrLoggable.h"


namespace frydom {

  // forward declarations
  namespace internal {
    class FrFEANodeBodyDistance;

    class FrFEANodeBase;

    class FrClumpGeometryBase;
  }

  class FrBody;

  class FrNode;

  class FrConstantForce;


  using FrClumpWeightBuoyancyForce = FrConstantForce;

  class FrClumpWeight : public FrObject, public FrLoggable<FrOffshoreSystem> {

   public:

    enum CLUMP_GEOMETRY {
      CYLINDER
    };

    FrClumpWeight(const std::string &name, FrOffshoreSystem *system);

    ~FrClumpWeight() = default;

    void SetDryMass(const double &mass);

    void SetSubmergedMass(const double &mass);

    void Attach(std::shared_ptr<internal::FrFEANodeBase> fea_node, const double &distance);

    void SetAsCylinder(const double &radius, const double &height);

    void SetMorisonCoefficients(const double &normal_Cd, const double &axial_Cd,
                                const double &normal_Cm, const double &axial_Cm);

    void Initialize() override;

    void InitializeBody();

    void InitializeLink();

    void InitializeNode();

    void InitializeMorison();

    void InitializeBuoyancy();

    std::shared_ptr<internal::FrFEANodeBase> GetFEANode();

    double GetConstraintDistance() const;

    void SetBodyNodePositionInWorld(const Position &position, FRAME_CONVENTION fc);

   private:
    void DefineLogMessages() override;


   private:
    struct MorisonCoeffs {
      double normal_Cd = 0.;
      double axial_Cd = 0.;
      double normal_Cm = 0.;
      double axial_Cm = 0.;
    };

   private:
    std::shared_ptr<internal::FrFEANodeBase> m_fea_node;
    std::shared_ptr<FrBody> m_body;
    std::shared_ptr<FrNode> m_body_node;

    std::shared_ptr<internal::FrFEANodeBodyDistance> m_constraint;
    double m_distance_to_node;

    bool m_is_dry_mass;
    double m_mass;

    MorisonCoeffs m_morison_coeffs;

    std::unique_ptr<internal::FrClumpGeometryBase> m_geometry;

    std::shared_ptr<FrClumpWeightBuoyancyForce> m_hydrostatic_force;

  };

  std::shared_ptr<FrClumpWeight>
  make_clump_weight(const std::string &name, FrOffshoreSystem *system);


  namespace internal {

    class FrClumpGeometryBase {

     public:
      virtual Position GetRelativePositionForNodeNWU() const = 0;

      double GetVolume() const;

      FrInertiaTensor GetInertiaTensor(const double &mass);

     protected:
      std::unique_ptr<Fr3DGeometryBase> m_geometry;
    };


    class FrClumpCylinderGeometry : public FrClumpGeometryBase {
     public:
      FrClumpCylinderGeometry(const double &radius, const double &height);

      Position GetRelativePositionForNodeNWU() const override;

    };


  }  // end namespace frydom::internal

}  // end namespace frydom


#endif //FRYDOM_FRCLUMPWEIGHT_H
