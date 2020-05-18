//
// Created by frongere on 26/02/2020.
//

#include "FrClumpWeight.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/core/body/FrBodyEasy.h"
#include "frydom/core/common/FrNode.h"
#include "frydom/core/force/FrConstantForce.h"
#include "frydom/logging/FrTypeNames.h"
#include "frydom/environment/FrEnvironment.h"

#include "frydom/cable/fea/FrFEALink.h"



namespace frydom {

  FrClumpWeight::FrClumpWeight(const std::string &name, FrOffshoreSystem *system) :
      FrLoggable(name, TypeToString(this), system),
      m_body(system->NewBody(name + "_body")),
      m_body_node(nullptr),
      m_fea_node(nullptr),
      m_constraint(std::make_shared<internal::FrFEANodeBodyDistance>()),
      m_distance_to_node(0.),
      m_is_dry_mass(false),
      m_mass(0.),
      m_morison_coeffs(),
      m_geometry(std::make_unique<internal::FrClumpCylinderGeometry>(1., 1.)),
      m_hydrostatic_force(nullptr) {}

  FrClumpWeight::~FrClumpWeight() {
    // Removing link and body from system
//    GetSystem()->Remove(m_body);
//    // TODO: Remove also the constraint !!
//    assert(false);
  }

  void FrClumpWeight::SetDryMass(const double &mass) {
    m_mass = mass;
    m_is_dry_mass = true;
  }

  void FrClumpWeight::SetSubmergedMass(const double &mass) {
    m_mass = mass;
    m_is_dry_mass = false;
  }


  void FrClumpWeight::SetAsCylinder(const double &radius, const double &height) {
    m_geometry = std::make_unique<internal::FrClumpCylinderGeometry>(radius, height);
  }

  void FrClumpWeight::SetMorisonCoefficients(const double &normal_Cd, const double &axial_Cd,
                                             const double &normal_Cm, const double &axial_Cm) {
    m_morison_coeffs.normal_Cd = normal_Cd;
    m_morison_coeffs.axial_Cd = axial_Cd;
    m_morison_coeffs.normal_Cm = normal_Cm;
    m_morison_coeffs.axial_Cm = axial_Cm;
  }

  void FrClumpWeight::Attach(std::shared_ptr<internal::FrFEANodeBase> fea_node, const double &distance) {
    m_fea_node = fea_node;
    m_distance_to_node = distance;
  }

  void FrClumpWeight::Initialize() {

    InitializeBody();

    InitializeNode();

//    InitializeConstraint();

    InitializeMorison(); // TODO: remettre

    InitializeBuoyancy();

    InitializeAsset();

  }

  void FrClumpWeight::InitializeBody() {
//    GetSystem()->Add(m_body);
    m_body->SetInertiaTensor(m_geometry->GetInertiaTensor(m_mass));
    m_body->LogThis(false); // This is the clump weight that have to be logged...
  }

  void FrClumpWeight::InitializeConstraint() {

//    auto node_position = internal::Vector3dToChVector(m_body_node->GetPositionInWorld(NWU));

    m_constraint->Initialize(m_fea_node, m_body_node, m_distance_to_node);

  }

  void FrClumpWeight::InitializeNode() {
    auto position = m_geometry->GetRelativePositionForNodeNWU();
    m_body_node = m_body->NewNode(GetName() + "_node");
    m_body_node->SetPositionInBody(position, NWU);
    m_body_node->LogThis(false);
  }

  void FrClumpWeight::InitializeMorison() {
    std::cerr << "Initialiser morison" << std::endl;
    // TODO
  }

  void FrClumpWeight::InitializeBuoyancy() {

    if (m_is_dry_mass) {
      // Here, a buoyancy constant force is applied to get the
      auto buoyancy_node = m_body->NewNode("buoyancy_node");
      buoyancy_node->LogThis(false);

      double bforce = m_geometry->GetVolume()
                      * GetSystem()->GetEnvironment()->GetFluidDensity(WATER) // FIXME: on pourrait mettre a jour...
                      * GetSystem()->GetGravityAcceleration();

      m_hydrostatic_force = make_constant_force(
          GetName() + "_buoyancy_force",
          buoyancy_node,
          FrClumpWeightBuoyancyForce::ABSOLUTE,
          {0., 0., bforce},
          NWU
      );
    }
  }

  void FrClumpWeight::InitializeAsset() {
    if (auto cylinder = dynamic_cast<internal::FrClumpCylinderGeometry*>(m_geometry.get())) {
      makeItCylinder(m_body, cylinder->GetRadius(), cylinder->GetHeight(), m_mass);
    }
  }

  std::shared_ptr<internal::FrFEANodeBase> FrClumpWeight::GetFEANode() {
    return m_fea_node;
  }

  double FrClumpWeight::GetConstraintDistance() const {
    return m_distance_to_node;
  }

  void FrClumpWeight::SetBodyNodePositionInWorld(const Position &position, FRAME_CONVENTION fc) {
    m_body_node->SetPositionInWorld(position, fc);
  }

  void FrClumpWeight::DefineLogMessages() {
    // TODO
  }

  void FrClumpWeight::Compute(double time) {
    // Nothing to do
  }

  std::shared_ptr<FrBody> FrClumpWeight::GetBody() {
    return m_body;
  }

  std::shared_ptr<FrClumpWeight>
  make_clump_weight(const std::string &name, FrOffshoreSystem *system) {
    auto clump_weight = std::make_shared<FrClumpWeight>(name, system);
    system->Add(clump_weight);
    return clump_weight;
  }


  namespace internal {

    double FrClumpGeometryBase::GetVolume() const {
      return m_geometry->GetVolume();
    }

    FrInertiaTensor FrClumpGeometryBase::GetInertiaTensor(const double &mass) {
      auto tensor = m_geometry->GetUnitInertiaTensor();
      tensor.Scale(mass);
      return tensor;
    }

    FrClumpCylinderGeometry::FrClumpCylinderGeometry(const double &radius, const double &height) {
      m_geometry = std::make_unique<FrCylinder>(radius, height);
    }

    Position FrClumpCylinderGeometry::GetRelativePositionForNodeNWU() const {
      return {0., 0., dynamic_cast<FrCylinder *>(m_geometry.get())->GetRadius()};
    }

    double FrClumpCylinderGeometry::GetRadius() const {
      return dynamic_cast<FrCylinder*>(m_geometry.get())->GetRadius();
    }

    double FrClumpCylinderGeometry::GetHeight() const {
      return dynamic_cast<FrCylinder*>(m_geometry.get())->GetHeight();
    }




    std::shared_ptr<FrFEANodeBodyDistance> GetClumpWeightConstraint(std::shared_ptr<FrClumpWeight> clump_weight) {
      return clump_weight->m_constraint;
    }

//    std::shared_ptr<FrBody> GetClumpWeightBody(std::shared_ptr<FrClumpWeight> clump_weight) {
//      return clump_weight->m_body;
//    }

  } // end namespace frydom::internal


}  // end namespace frydom
