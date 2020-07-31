//
// Created by lletourn on 05/03/19.
//

#include "FrFEACable.h"

#include <chrono/fea/ChBeamSectionCosserat.h>
#include <chrono/physics/ChLoadContainer.h>
#include <chrono/fea/ChContactSurfaceNodeCloud.h>
#include <chrono/fea/ChVisualizationFEAmesh.h>

#include "frydom/core/common/FrNode.h"
#include "frydom/core/math/bspline/FrBSpline.h"
#include "frydom/logging/FrTypeNames.h"
#include "frydom/cable/common/FrCableShapeInitializer.h"
#include "frydom/cable/mooring_components/FrClumpWeight.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/cable/common/FrCableProperties.h"
#include "FrFEACableElement.h"
#include "FrFEACableLoads.h"
#include "FrFEACableBuilder.h"
#include "FrFEALink.h"
#include "FrFEACableSection.h"
#include "frydom/core/FrOffshoreSystem.h"

namespace frydom {


  FrFEACable::FrFEACable(const std::string &name,
                         const std::shared_ptr<FrNode> &startingNode,
                         const std::shared_ptr<FrNode> &endingNode,
                         const std::shared_ptr<FrCableProperties> &properties,
                         double unstretched_length,
                         unsigned int nb_nodes) :
      m_start_link_type(SPHERICAL),
      m_end_link_type(SPHERICAL),
      m_nb_nodes(nb_nodes),
      FrCableBase(startingNode, endingNode, properties, unstretched_length),
      FrFEAMesh(name,
                TypeToString(this),
                startingNode->GetBody()->GetSystem(),
                std::make_shared<internal::FrFEACableBase>(name, this, startingNode->GetBody()->GetSystem())) {}

  Force FrFEACable::GetTension(const double &s, FRAME_CONVENTION fc) const {
    // TODO
  }

  /*
  Force FrFEACable::GetForceAtStartLink(FRAME_CONVENTION fc) {
    auto cable_base = GetFrFEACableBase();
    auto start_link = cable_base->GetStartLink();
    auto start_force = internal::ChVectorToVector3d<Force>(start_link->Get_react_force());

    if (IsNED(fc)) internal::SwapFrameConvention(start_force);

    // FIXME: il faut voir la convention qu'on veut adopter. Une possibilite est d'avoir la force appliquee par le
    // cable sur son environnement... Certainement des ajustements ici...
    return start_force;
  }

  Force FrFEACable::GetForceAtEndLink(FRAME_CONVENTION fc) {
    auto cable_base = GetFrFEACableBase();
    auto end_link = cable_base->GetEndLink();
    auto end_force = internal::ChVectorToVector3d<Force>(end_link->Get_react_force());

    if (IsNED(fc)) internal::SwapFrameConvention(end_force);

    // FIXME: il faut voir la convention qu'on veut adopter. Une possibilite est d'avoir la force appliquee par le
    // cable sur son environnement... Certainement des ajustements ici...
    return end_force;
  }
   */

  Position FrFEACable::GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const {
    // TODO
  }

  void FrFEACable::Initialize() {
    m_chrono_mesh->Initialize();

    InitializeClumpWeights();

    //DefineLogMessages(); // TODO: voir si on appelle ca ici avec les autres classes ...
  }

  void FrFEACable::InitializeClumpWeights() {

    for (auto &cw : m_clump_weights) {

      auto clump_weight = cw.m_clump_weight;
      double abscissa = cw.m_abscissa;
      double distance = cw.m_distance_to_node;

      // Attaching the clump to the nearest node on the line
      // TODO: a terme, il conviendrait d'inserer un point de controle a cette absc
      auto fea_node = GetNearestFEANode(abscissa / GetUnstretchedLength());
      clump_weight->Attach(fea_node, distance);

      // Get the position of the node
      Position position = internal::ChVectorToVector3d<Position>(fea_node->GetPos());
      position.z() -= clump_weight->GetConstraintDistance();

      clump_weight->SetBodyNodePositionInWorld(position, NWU);

      clump_weight->InitializeConstraint();
    }

  }

  void FrFEACable::StepFinalize() {
    // TODO
  }

  void FrFEACable::Relax() {
    // TODO
  }

  double FrFEACable::GetStaticResidual() {
    // TODO
  }

  unsigned int FrFEACable::GetNbNodes() const {
    return m_nb_nodes;
  }

  FrFEACable::FEA_BODY_CONSTRAINT_TYPE FrFEACable::GetStartLinkType() const {
    return m_start_link_type;
  }

  void FrFEACable::SetStartLinkType(FEA_BODY_CONSTRAINT_TYPE ctype) {
    m_start_link_type = ctype;
    GetFrFEACableBase()->SetStartLinkConstraint(ctype);
  }

  FrFEACable::FEA_BODY_CONSTRAINT_TYPE FrFEACable::GetEndLinkType() const {
    return m_end_link_type;
  }

  void FrFEACable::SetEndLinkType(FEA_BODY_CONSTRAINT_TYPE ctype) {
    m_end_link_type = ctype;
    GetFrFEACableBase()->SetEndLinkConstraint(ctype);
  }

  // FIXME: ici on prend le FEA node le plus proche de l'abscisse s specifiee...
  std::shared_ptr<FrClumpWeight>
  FrFEACable::AddClumpWeight(const std::string &name, const double &s, const double &distance) {
    assert(0. <= s && s <= GetUnstretchedLength());

    auto clump_weight = make_clump_weight(name, GetSystem());
    m_clump_weights.emplace_back(ClumpWeight_(clump_weight, s, distance));

    return clump_weight;
  }

  std::shared_ptr<internal::FrFEANodeBase> FrFEACable::GetNearestFEANode(const double &s) {
    return GetFrFEACableBase()->GetNearestFEANode(s);
  }

  //##CC
  Force FrFEACable::GetForceStartNodeInWorld(FRAME_CONVENTION fc) {

    auto base_cable = GetFrFEACableBase();
    auto start_link = base_cable->GetStartLink();

    auto force = internal::ChVectorToVector3d<Force>(start_link->Get_react_force());
    if (IsNED(fc)) internal::SwapFrameConvention<Force>(force);
    return m_startingNode->GetFrameWRT_COG_InBody().ProjectVectorFrameInParent(force, fc);
  }

  Force FrFEACable::GetForceEndNodeInWorld(FRAME_CONVENTION fc) {

    auto base_cable = GetFrFEACableBase();
    auto end_link = base_cable->GetEndLink();

    auto force = internal::ChVectorToVector3d<Force>(end_link->Get_react_force());
    if (IsNED(fc)) internal::SwapFrameConvention<Force>(force);
    return m_endingNode->GetFrameWRT_COG_InBody().ProjectVectorFrameInParent(force, fc);
  }

  Force FrFEACable::GetForceEndNodeInBody(FRAME_CONVENTION fc) {

    auto base_cable = GetFrFEACableBase();
    auto end_link = base_cable->GetEndLink();

    auto force = internal::ChVectorToVector3d<Force>(end_link->Get_react_force());
    if (IsNED(fc)) internal::SwapFrameConvention<Force>(force);
    return force;
  }

  void FrFEACable::DefineLogMessages() {

    auto msg = NewMessage("Cable states", "Data of cable");


    msg->AddField<double>("Time", "s", "Simulation time",
                          [this]() { return GetSystem()->GetTime(); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>(
        "StartingNode", "m", fmt::format("position of the starting node in world reference frame in {}", GetLogFC()),
        [this]() { return m_startingNode->GetPositionInWorld(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>(
        "ForceStartNodeInWorld", "N", fmt::format("force at the starting point in world reference frame in {}", GetLogFC()),
        [this]() { return GetForceStartNodeInWorld(GetLogFC()); });

    //msg->AddField<Eigen::Matrix<double, 3, 1>>("Start_tension_vector", "N",
    //                                           "Tension vector at start node",
    //                                           [this]() { return GetForceAtStartLink(NWU); });

    msg->AddField<double>("Start_tension", "N", "Tension at start node",
                          [this]() { return GetForceStartNodeInWorld(NWU).norm(); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>(
        "EndingNode", "m", fmt::format("position of the ending node in world reference frame in {}", GetLogFC()),
        [this]() { return m_endingNode->GetPositionInWorld(GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>(
        "ForceEndNodeInWorld", "N", fmt::format("force at the ending point in world reference frame in {}", GetLogFC()),
        [this]() { return GetForceEndNodeInWorld(GetLogFC()); });

    //msg->AddField<Eigen::Matrix<double, 3, 1>>("End_tension_vector", "N",
    //                                           "Tension vector at end node",
    //                                           [this]() { return GetForceAtEndLink(NWU); });

    msg->AddField<double>("End_tension", "N",
                          "Tension at end node",
                          [this]() { return GetForceEndNodeInWorld(NWU).norm(); });
  }

  void FrFEACable::BuildCache() {
    // TODO
  }

  internal::FrFEACableBase *FrFEACable::GetFrFEACableBase() {
    return dynamic_cast<internal::FrFEACableBase *>(m_chrono_mesh.get());
  }


  std::shared_ptr<FrFEACable> make_fea_cable(const std::string &name,
                                             const std::shared_ptr<FrNode> &startingNode,
                                             const std::shared_ptr<FrNode> &endingNode,
                                             const std::shared_ptr<FrCableProperties> &properties,
                                             double unstretched_length,
                                             unsigned int nb_elements) {

    // This cable is empty
    auto cable = std::make_shared<FrFEACable>(name,
                                              startingNode,
                                              endingNode,
                                              properties,
                                              unstretched_length,
                                              nb_elements);

    startingNode->GetBody()->GetSystem()->Add(cable);

    return cable;
  }


  namespace internal {

    FrFEACableBase::FrFEACableBase(const std::string& name, FrFEACable *cable, FrOffshoreSystem* system) :
        m_bspline_order(2),
        m_start_link(std::make_shared<FrFEALinkBase>(name+"_start_link", system)),
        m_end_link(std::make_shared<FrFEALinkBase>(name+"_end_link", system)),
        FrFEAMeshBase(cable) {
    }

    std::shared_ptr<FrFEALinkBase> FrFEACableBase::GetStartLink() {
      return m_start_link;
    }

    std::shared_ptr<FrFEALinkBase> FrFEACableBase::GetEndLink() {
      return m_end_link;
    }

    void FrFEACableBase::Initialize() {

      BuildProperties();

      InitializeShape();

      InitializeLoads();

      InitializeLinks();

      InitializeContacts();

      InitializeAssets();

      SetupInitial();

    }

    void FrFEACableBase::BuildProperties() {
      auto props = GetFEACable()->GetProperties();

      // Constitutive material
      auto elasticity = std::make_shared<chrono::fea::ChElasticityCosseratSimple>();
      elasticity->SetYoungModulus(props->GetYoungModulus());
      // FIXME: voir a importer cela aussi depuis les props
      elasticity->SetGshearModulus(props->GetYoungModulus() * 0.3);
      elasticity->SetBeamRaleyghDamping(props->GetRayleighDamping());

      // Section definition
      m_section = std::make_shared<FrFEACableSection>(elasticity);
      m_section->SetDensity(props->GetDensity());
      m_section->SetAsCircularSection(props->GetDiameter());
      m_section->SetNormalAddedMassCoeff(props->GetTransverseAddedMassCoefficient());
      m_section->SetNormalDragCoeff(props->GetTransverseDragCoefficient());
      m_section->SetVIVAmpFactor(props->GetVIVAmpFactor()); // TODO: a inclure dans le pptes de cable...

    }

    void FrFEACableBase::InitializeShape() {

      // TODO: voir ChElementBeam::SetRestLength qui permet de forcer la longueur a vie des elements !!! Ca permettra
      // d'avoir des elements precompresses ou preentendus... et de ne pas utiliser le cables catenaires en
      // initialisation avec une elasticite nulle...

      auto fea_cable = GetFEACable();
      double unstretched_length = fea_cable->GetUnstretchedLength();

      // FIXME: pourquoi doit on entrer l'environnement alors qu'on y a acces depuis fea_cable ??
      auto shape_initializer =
          FrCableShapeInitializer::Create(m_frydom_mesh->GetName(), fea_cable,
                                          fea_cable->GetSystem()->GetEnvironment());

      unsigned int n = fea_cable->GetNbNodes();

      // Building a vector of points on the neutral line of the cable at initial position (approx. equilibrum)
      std::vector<double> uvec = mathutils::linspace(0., unstretched_length, n);
      std::vector<bspline::Point<3>> neutral_line_points(n);
      for (unsigned int i = 0; i < n; i++) {
        neutral_line_points[i] = shape_initializer->GetPosition(uvec[i], NWU);
        // FIXME:  on evalue dans une position detendue. Voir la remarque au debut...
      }

      // Interpolating with BSpline
      // FIXME: l'ordre est hard code...
      auto bspline = bspline::internal::FrBSplineTools<2>::BSplineInterpFromPoints<3>(neutral_line_points,
                                                                                      m_control_points_abscissa);

      // Building the shape with the FEABuilder
      FrFEACableBuilder builder;
      builder.Build(this, m_section, bspline);

    }

    void FrFEACableBase::InitializeLoads() {
      // Creating a load container for the cable
      auto load_container = std::make_shared<chrono::ChLoadContainer>();
      system->Add(load_container);

      // Adding loads on each element newly created by the builder
      for (auto &element : GetElements()) { // TODO : faire des iterateurs d'elements !!!
        auto el = std::dynamic_pointer_cast<FrFEACableElementBase>(element);
        auto hydro_load = std::make_shared<internal::FrFEACableHydroLoad>(el);
        hydro_load->SetCable(GetFEACable());
        load_container->Add(hydro_load);
      }
    }

    void FrFEACableBase::InitializeLinks() {

      auto fea_cable = GetFEACable();

      // Starting hinge
      auto starting_body = internal::GetChronoBody(fea_cable->GetStartingNode()->GetBody());
      auto start_ch_frame = internal::FrFrame2ChFrame(fea_cable->GetStartingNode()->GetFrameInBody());

      m_start_link->Initialize(GetStartNodeFEA(),
                               starting_body,
                               true,
                               chrono::ChFrame<double>(),
                               start_ch_frame);

      // Ending hinge
      auto ending_body = internal::GetChronoBody(fea_cable->GetEndingNode()->GetBody());
      auto end_ch_frame = internal::FrFrame2ChFrame(fea_cable->GetEndingNode()->GetFrameInBody());
      FrFrame feaFrame;
      feaFrame.RotZ_RADIANS(MU_PI, NWU, false); // ending_node_fea comes from the opposite direction

      m_end_link->Initialize(GetEndNodeFEA(),
                             ending_body,
                             true,
                             internal::FrFrame2ChFrame(feaFrame),
                             end_ch_frame);

      // Set constraints on the links
      SetStartLinkConstraint(fea_cable->GetStartLinkType());
      SetEndLinkConstraint(fea_cable->GetEndLinkType());

    }

    void FrFEACableBase::InitializeContacts() {

      // TODO: il sera certainement interessant de pouvoir

      auto surface_material = std::make_shared<chrono::ChMaterialSurfaceSMC>();
      surface_material->SetYoungModulus(2e12f);
      surface_material->SetFriction(0.3f);
      surface_material->SetRestitution(0.0f);
      surface_material->SetAdhesion(0);
      surface_material->SetKn(2e12);
      surface_material->SetGn(1e6);

      auto contact_surface = std::make_shared<chrono::fea::ChContactSurfaceNodeCloud>();
      AddContactSurface(contact_surface);
      contact_surface->AddAllNodes(GetFEACable()->GetProperties()->GetRadius());
      contact_surface->SetMaterialSurface(surface_material);

    }

    void FrFEACableBase::InitializeAssets() {

      double cable_diam = GetFEACable()->GetProperties()->GetDiameter();

      auto elements_assets = std::make_shared<chrono::fea::ChVisualizationFEAmesh>(*this);
      elements_assets->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_TX);
      // TODO: remettre en place !
//        elements_assets->SetColorscaleMinMax(-fea_cable->GetBreakingTension(),
//                                             fea_cable->GetBreakingTension()); //1799620
      elements_assets->SetSmoothFaces(true);
      elements_assets->SetWireframe(false);
      // TODO : mettre en place un mode "BIG VIZ" qui grossit le cable et tout
//      m_section->SetDrawCircularRadius(cable_diam * 0.5);
      m_section->SetDrawCircularRadius(cable_diam);
      ChMesh::AddAsset(elements_assets);

      // Assets for the nodes
      auto node_assets = std::make_shared<chrono::fea::ChVisualizationFEAmesh>(*this);
//        node_assets->SetFEMglyphType(chrono::fea::ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
      node_assets->SetFEMglyphType(chrono::fea::ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
      node_assets->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_NONE);
      node_assets->SetSymbolsThickness(cable_diam * 5);
      node_assets->SetSymbolsScale(0.001);
      node_assets->SetZbufferHide(false);
      ChMesh::AddAsset(node_assets);

    }

    void FrFEACableBase::SetupInitial() {
      chrono::fea::ChMesh::SetupInitial();
    }

    void FrFEACableBase::SetStartLinkConstraint(FrFEACable::FEA_BODY_CONSTRAINT_TYPE ctype) {
      SetLinkConstraint(ctype, m_start_link.get());
    }

    void FrFEACableBase::SetEndLinkConstraint(FrFEACable::FEA_BODY_CONSTRAINT_TYPE ctype) {
      SetLinkConstraint(ctype, m_end_link.get());
    }

    void FrFEACableBase::SetLinkConstraint(FrFEACable::FEA_BODY_CONSTRAINT_TYPE ctype, FrFEALinkBase *link) {
      switch (ctype) {
        case FrFEACable::FEA_BODY_CONSTRAINT_TYPE::FREE:
          link->SetConstrainedCoords(false, false, false, false, false, false);
          break;
        case FrFEACable::FEA_BODY_CONSTRAINT_TYPE::SPHERICAL:
          link->SetConstrainedCoords(true, true, true, false, false, false);
          break;
        case FrFEACable::FEA_BODY_CONSTRAINT_TYPE::FIXED:
          link->SetConstrainedCoords(true, true, true, true, true, true);
          break;
      }
    }

    FrFEACable *FrFEACableBase::GetFEACable() {
      return dynamic_cast<FrFEACable *>(m_frydom_mesh);
    }

    std::shared_ptr<FrFEANodeBase> FrFEACableBase::GetStartNodeFEA() {
      return std::dynamic_pointer_cast<FrFEANodeBase>(GetNodes().front());
    }

    std::shared_ptr<FrFEANodeBase> FrFEACableBase::GetEndNodeFEA() {
      return std::dynamic_pointer_cast<FrFEANodeBase>(GetNodes().back()); // FIXME: ne fonctionne pas !!
    }

    std::shared_ptr<internal::FrFEANodeBase> FrFEACableBase::GetNearestFEANode(const double &s) {
      assert(0 <= s && s <= GetFEACable()->GetUnstretchedLength()); // TODO: verifier

      unsigned int idx = 0;
      double delta = 1.;
      for (unsigned int i = 0; i < m_control_points_abscissa.size(); i++) {
        double delta_tmp = std::fabs(s - m_control_points_abscissa[i]);
        if (delta_tmp < delta) {
          delta = delta_tmp;
          idx = i;
        }
      }

      return std::dynamic_pointer_cast<internal::FrFEANodeBase>(GetNode(idx));

    }

    std::shared_ptr<FrFEACableBase> GetChronoFEAMesh(std::shared_ptr<FrFEACable> cable) {
      return std::dynamic_pointer_cast<FrFEACableBase>(cable->m_chrono_mesh);
    }

  }  // end namespace frydom::internal

} // end namespace frydom
