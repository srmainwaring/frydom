//
// Created by lletourn on 05/03/19.
//

//#include <MathUtils/VectorGeneration.h>
//
//#include <chrono/fea/ChBeamSection.h>
//#include <chrono/physics/ChLinkMate.h>
//#include <chrono/fea/ChNodeFEAxyzrot.h>
//#include <chrono/fea/ChVisualizationFEAmesh.h>
//#include <chrono/fea/ChContactSurfaceNodeCloud.h>
//#include <chrono/fea/ChContactSurfaceMesh.h>
//#include <chrono/physics/ChLoadContainer.h>
//#include <chrono/fea/ChBeamSectionCosserat.h>
//
//
#include "FrFEACable.h"
//
//#include "frydom/core/math/BSpline/FrBSpline.h"
//
//#include "FrFEACableLoads.h"
//#include "frydom/cable/common/FrCableProperties.h"
//

#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"

#include "frydom/logging/FrTypeNames.h"
//
//#include "frydom/cable/common/FrCableShapeInitializer.h"

namespace frydom {

  namespace internal {

    FrFEACableBase::FrFEACableBase(FrFEACable *cable) : m_frydom_cable(cable) {
// TODO
    }


  }  // end namespace frydom::internal

//  void FreeNode(bool start, bool end) {
//
//  }


  Force FrFEACable::GetTension(const double &s, FRAME_CONVENTION fc) const {
    // TODO
  }

  Position FrFEACable::GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const {
// TODO
  }

  FrFEACable::FrFEACable(const std::string &name,
                         const std::shared_ptr<FrNode> &startingNode,
                         const std::shared_ptr<FrNode> &endingNode,
                         const std::shared_ptr<FrCableProperties> &properties,
                         double unstretched_length,
                         unsigned int nb_elements) :
      FrLoggable(name, TypeToString(this), startingNode->GetBody()->GetSystem()),
      FrFEAMesh(),
      FrCableBase(startingNode, endingNode, properties, unstretched_length),
      m_nb_elements(nb_elements) {
  }

  void FrFEACable::Relax() {
// TODO
  }

  double FrFEACable::GetStaticResidual() {
    // TODO
  }

  void FrFEACable::Compute(double time) {
// TODO
  }

  void FrFEACable::DefineLogMessages() {
// TODO
  }

  void FrFEACable::BuildCache() {
// TODO
  }


  std::shared_ptr<FrFEACable> make_fea_cable(const std::string &name,
                                             const std::shared_ptr<FrNode> &startingNode,
                                             const std::shared_ptr<FrNode> &endingNode,
                                             const std::shared_ptr<FrCableProperties> &properties,
                                             double unstretched_length,
                                             unsigned int nb_elements) {
    auto cable = std::make_shared<FrFEACable>(name,
                                              startingNode,
                                              endingNode,
                                              properties,
                                              unstretched_length,
                                              nb_elements);
    startingNode->GetBody()->GetSystem()->Add(cable);
    return cable;
  }




//
//    FrFEACableBase::FrFEACableBase(FrFEACable *cable) : chrono::fea::ChMesh(), m_frydomCable(cable) {
//
//      m_startingHinge = std::make_shared<chrono::ChLinkMateGeneric>();
//      m_endingHinge = std::make_shared<chrono::ChLinkMateGeneric>();
////      m_section = std::make_shared<chrono::fea::ChBeamSectionAdvanced>();
//
//    }
//
//    void FrFEACableBase::InitializeContact() {
//
//      auto surface_material = std::make_shared<chrono::ChMaterialSurfaceSMC>();
//      surface_material->SetYoungModulus(2e12f);
//      surface_material->SetFriction(0.3f);
//      surface_material->SetRestitution(0.0f);
//      surface_material->SetAdhesion(0);
//      surface_material->SetKn(2e12);
//      surface_material->SetGn(1e6);
//
//
////      auto contact_surface = std::make_shared<chrono::fea::ChContactSurfaceMesh>();
////      AddContactSurface(contact_surface);
////      contact_surface->AddFacesFromBoundary(0.008);
////      contact_surface->SetMaterialSurface(surface_material);
//
//
//      auto contact_surface = std::make_shared<chrono::fea::ChContactSurfaceNodeCloud>();
//      AddContactSurface(contact_surface);
//      contact_surface->AddAllNodes(m_frydomCable->GetProperties()->GetRadius());
//      contact_surface->SetMaterialSurface(surface_material);
//
//    }
//
//    void FrFEACableBase::InitializeHydrodynamicLoads() {
//
//      // Loads must be added to a load container
//      auto load_container = std::make_shared<chrono::ChLoadContainer>();
//      GetSystem()->Add(load_container);
//
//      for (const auto &element : GetElements()) {
//        // FIXME: changer, seulement pour debug
////        if (auto loadable = std::dynamic_pointer_cast<internal::FrElementBeamIGA>(element)) {
////        if (auto loadable = std::dynamic_pointer_cast<chrono::ChLoadableU>(element)) { // FIXME: on veut caster en
//        if (auto loadable = std::dynamic_pointer_cast<FrElementBeamIGA>(element)) { // FIXME: on veut caster en
//          // Adding buoyancy
//          auto load = std::make_shared<FrBuoyancyLoad>(m_frydomCable, loadable);
//          load_container->Add(load);
//        } else {
//          std::cerr << "Cannot cast" << std::endl;
//          exit(EXIT_FAILURE);
//        }
//
//      }
//
//
//
////      auto buoyancy_load = std::make_shared<FrBuoyancyLoader>(m_frydomCable->GetSystem());
//
//      // Creer les autres loads !!
//
//
//    }
//
//    void FrFEACableBase::InitializeLinks() {
//
//      // Starting hinge
//      auto starting_body = m_frydomCable->GetStartingNode()->GetBody()->m_chronoBody;
//      auto start_ch_frame = internal::FrFrame2ChFrame(m_frydomCable->GetStartingNode()->GetFrameInBody());
//
//      m_startingHinge->Initialize(m_starting_node_fea,
//                                  starting_body,
//                                  true,
//                                  chrono::ChFrame<double>(),
//                                  start_ch_frame);
//
//      // Ending hinge
//      auto ending_body = m_frydomCable->GetEndingNode()->GetBody()->m_chronoBody;
//      auto end_ch_frame = internal::FrFrame2ChFrame(m_frydomCable->GetEndingNode()->GetFrameInBody());
//      FrFrame feaFrame;
//      feaFrame.RotZ_RADIANS(MU_PI, NWU, false); // ending_node_fea comes from the opposite direction
//
//      m_endingHinge->Initialize(m_ending_node_fea,
//                                ending_body,
//                                true,
//                                internal::FrFrame2ChFrame(feaFrame),
//                                end_ch_frame);
//
//      // Define the constraints on the hinges.
//      SetHingeConstraints();
//
//    }
//
//    void FrFEACableBase::SetHingeConstraints() {
//
//      switch (m_frydomCable->GetStartingHingeType()) {
//        case FrFEACable::BOUNDARY_CONSTRAINT_TYPE::NONE:
//          m_startingHinge->SetConstrainedCoords(false, false, false, false, false, false);
//          break;
//        case FrFEACable::BOUNDARY_CONSTRAINT_TYPE::SPHERICAL:
//          m_startingHinge->SetConstrainedCoords(true, true, true, false, false, false);
//          break;
//        case FrFEACable::BOUNDARY_CONSTRAINT_TYPE::CONSTRAINED:
//          m_startingHinge->SetConstrainedCoords(true, true, true, true, true, true);
//          break;
//      }
//
//      switch (m_frydomCable->GetEndingHingeType()) {
//        case FrFEACable::BOUNDARY_CONSTRAINT_TYPE::NONE:
//          m_endingHinge->SetConstrainedCoords(false, false, false, false, false, false);
//          break;
//        case FrFEACable::BOUNDARY_CONSTRAINT_TYPE::SPHERICAL:
//          m_endingHinge->SetConstrainedCoords(true, true, true, false, false, false);
//          break;
//        case FrFEACable::BOUNDARY_CONSTRAINT_TYPE::CONSTRAINED:
//          m_endingHinge->SetConstrainedCoords(true, true, true, true, true, true);
//          break;
//      }
//
//    }
//
//    void FrFEACableBase::GenerateAssets() {
//      // Assets for the cable visualisation
//      if (m_drawCableElements) {
//        auto elements_assets = std::make_shared<chrono::fea::ChVisualizationFEAmesh>(*this);
//        elements_assets->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_TX);
//        elements_assets->SetColorscaleMinMax(-m_frydomCable->GetBreakingTension(),
//                                             m_frydomCable->GetBreakingTension()); //1799620
//        elements_assets->SetSmoothFaces(true);
//        elements_assets->SetWireframe(false);
//        m_frydomCable->GetSection()->SetDrawCircularRadius(m_frydomCable->GetDrawElementRadius());
//        ChMesh::AddAsset(elements_assets);
//      }
//
//      // Assets for the nodes
//      if (m_drawCableNodes) {
//        auto node_assets = std::make_shared<chrono::fea::ChVisualizationFEAmesh>(*this);
//        node_assets->SetFEMglyphType(
//            chrono::fea::ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS); // E_GLYPH_NODE_CSYS
//        node_assets->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_NONE);
//        node_assets->SetSymbolsThickness(m_frydomCable->GetDrawNodeSize());
//        node_assets->SetSymbolsScale(0.01);
//        node_assets->SetZbufferHide(false);
//        ChMesh::AddAsset(node_assets);
//      }
//
//    }
//
//    void FrFEACableBase::Initialize() {
//      // Must be called after the ChMesh is initialized !
//
//      if (m_starting_node_fea == nullptr) {
//
//        auto nodes = GetNodes();
//
//        // FIXME: il faut renseigner de nouveau les noeuds aux frontieres !!!
//        m_starting_node_fea = std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(nodes.front());
//        m_ending_node_fea = std::dynamic_pointer_cast<chrono::fea::ChNodeFEAxyzrot>(nodes.back());
//
//        // Generate constraints between boundary nodes just defined and bodies
//        InitializeLinks();
//
////        if (m_frydomCable->GetBreakingTension() == 0.) {
////
//////          if (is_taut) {
//////            double tensionMax = (distanceBetweenNodes.norm() - m_frydomCable->GetUnstretchedLength()) *
//////                                m_frydomCable->GetProperties()->GetEA() / m_frydomCable->GetUnstretchedLength();
//////            m_frydomCable->SetBreakingTension(1.2 * tensionMax);
//////          } else {
//////            // TODO: remettre en place le mecanisme ...
////////            m_frydomCable->SetBreakingTension(1.2 * catenaryLine->GetMaxTension());
//////          }
////
////        }
//
//        // Set reference position of nodes as current position, for all nodes.
////        Relax();
//
//        InitializeHydrodynamicLoads(); // TODO: remettre en place
//
//        InitializeContact();
//
//        GenerateAssets();
//
//      }
//
//      // Initialize the Chrono FEA part
//      SetupInitial();
//
//    }
//
//    void FrFEACableBase::Update(double time, bool update_assets) {
//
////      UpdateForces(time);
//
//      chrono::fea::ChMesh::Update(time, update_assets);
//      m_frydomCable->Update(time);
//
//    }
//
//    Position FrFEACableBase::GetNodePositionInWorld(int index, double eta) {
//
//      chrono::ChVector<double> position;
//      chrono::ChQuaternion<double> quaternion;
//
//      auto element = std::dynamic_pointer_cast<FrElementBeamIGA>(GetElement(index));
//      element->EvaluateSectionFrame(eta, position, quaternion);
//
//      return internal::ChVectorToVector3d<Position>(position);
//    }
//
//    Force FrFEACableBase::GetTension(int index, double eta) {
//
//      chrono::ChVector<double> tension, torque;
//
//      auto element = std::dynamic_pointer_cast<FrElementBeamIGA>(GetElement(index));
//
//      element->EvaluateSectionForceTorque(eta, tension, torque); // FIXME: cette methode ne fait rien :/
//
//      chrono::ChVector<double> position;
//      chrono::ChQuaternion<double> quaternion;
//      element->EvaluateSectionFrame(eta, position, quaternion);
//
//      auto tangent = internal::ChVectorToVector3d<Direction>(quaternion.GetXaxis());
//      auto force = internal::ChVectorToVector3d<Force>(tension);
//
//      return force.dot(tangent) * tangent;
//
//    }
//
//
//  }  // end namespace frydom::internal
//
//  FrFEACable::FrFEACable(const std::string &name,
//                         const std::shared_ptr<frydom::FrNode> &startingNode,
//                         const std::shared_ptr<frydom::FrNode> &endingNode,
//                         const std::shared_ptr<FrCableProperties> &properties,
//                         double unstrainedLength,
//                         double rayleighDamping,
//                         unsigned int nbElements) :
//      FrLoggable(name, TypeToString(this), startingNode->GetBody()->GetSystem()),
//      FrFEAMesh(),
//      FrCableBase(startingNode, endingNode, properties, unstrainedLength),
//      m_rayleighDamping(rayleighDamping),
//      m_nbElements(nbElements) {
//
//    m_chronoCable = std::make_shared<internal::FrFEACableBase>(this);
//    LogThis(true);
//
//  }
//
//
//  void FrFEACable::SetRayleighDamping(double damping) {
//    m_rayleighDamping = damping;
//  }
//
//  double FrFEACable::GetRayleighDamping() const {
//    return m_rayleighDamping;
//  }
//
//  void FrFEACable::SetNumberOfElements(unsigned int nbElements) {
//    m_nbElements = nbElements;
//  }
//
//  unsigned int FrFEACable::GetNumberOfElements() const {
//    return m_nbElements;
//  }
//
//  void FrFEACable::SetTargetElementLength(double elementLength) {
//    assert(elementLength > 0. && elementLength < GetUnstretchedLength());
//    m_nbElements = static_cast<unsigned int>(int(floor(GetUnstretchedLength() / elementLength)));
//  }
//
//  void FrFEACable::DefineLogMessages() {
//
//    auto msg = NewMessage("State", "State message");
//
//    msg->AddField<double>("time", "s", "Current time of the simulation",
//                          [this]() { return GetSystem()->GetTime(); });
//
//    msg->AddField<double>("StrainedLength", "m", "Strained Length of the dynamic cable",
//                          [this]() { return GetStrainedLength(); });
//
//    msg->AddField<Eigen::Matrix<double, 3, 1>>
//        ("StartingNodeTension", "N", fmt::format("Starting node tension in world reference frame in {}", GetLogFC()),
//         [this]() { return GetTension(0., GetLogFC()); });
//
//    msg->AddField<Eigen::Matrix<double, 3, 1>>
//        ("EndingNodeTension", "N", fmt::format("Ending node tension in world reference frame in {}", GetLogFC()),
//         [this]() { return -GetTension(GetUnstretchedLength(), GetLogFC()); });
//
//  }
//
//  void FrFEACable::SetBreakingTension(double tension) {
//    m_maxTension = tension;
//  }
//
//  double FrFEACable::GetBreakingTension() const {
//    return m_maxTension;
//  }
//
//  void FrFEACable::SetDrawElementRadius(double radius) {
//    m_drawCableElementRadius = radius;
//  }
//
//  double FrFEACable::GetDrawElementRadius() {
//    return m_drawCableElementRadius;
//  }
//
//  void FrFEACable::SetDrawNodeSize(double size) {
//    m_drawCableNodeSize = size;
//  }
//
//  double FrFEACable::GetDrawNodeSize() const {
//    return m_drawCableElementRadius;
//  }
//
//  void FrFEACable::Initialize() {
//
//    // Building the section
//    BuildSection();
//
//    // Determining if the cable is taut or slack
//    bool is_taut = (m_endingNode->GetPositionInWorld(NWU) - m_startingNode->GetPositionInWorld(NWU)).norm()
//                   > m_unstretchedLength;
//
//    if (is_taut) {
//      InitializeTaut();
//    } else {
//      InitializeSlack();
//    }
//
//    m_chronoCable->Initialize();
//
//  }
//
//  void FrFEACable::BuildSection() {
//
//    m_elasticity = std::make_shared<chrono::fea::ChElasticityCosseratSimple>();
//    m_elasticity->SetYoungModulus(m_properties->GetYoungModulus());
//    m_elasticity->SetGshearModulus(0.3 * m_properties->GetYoungModulus()); // By default, 30% of the young modulus...
//    m_elasticity->SetBeamRaleyghDamping(m_properties->GetRayleighDamping());
//
//    m_section = std::make_shared<chrono::fea::ChBeamSectionCosserat>(m_elasticity);
//    m_section->SetDensity(m_properties->GetDensity());
//    m_section->SetAsCircularSection(m_properties->GetDiameter());
//
//  }
//
//  void FrFEACable::InitializeTaut() {
//
//    chrono::fea::ChBuilderBeamIGA builder;
//
//    builder.BuildBeam(m_chronoCable,
//                      m_section,
//                      m_nbElements,
//                      internal::Vector3dToChVector(m_startingNode->GetPositionInWorld(NWU)),
//                      internal::Vector3dToChVector(m_endingNode->GetPositionInWorld(NWU)),
//                      {0., 0., 1.},
//                      3);
//  }
//
//  void FrFEACable::InitializeSlack() {
//
//    // *
//    // Building a shape initializer that uses internally a catenary cable
//    // *
//    auto shape_initializer = FrCableShapeInitializer::Create(this, GetSystem()->GetEnvironment());
//
//    // *
//    // Fitting the shape with a bspline
//    // *
//
//    // Building the set of points to interpolate (equal to the number of control points)
//    int p = 3; // BSpline order
//    int nb_ctrl_points = m_nbElements + p;
//    std::vector<double> u_vec = mathutils::linspace(0., m_unstretchedLength, nb_ctrl_points);
//    std::vector<bspline::Point<3>> points;
//    points.reserve(nb_ctrl_points);
//    for (unsigned int i = 0; i < nb_ctrl_points; i++) {
//      points.push_back(shape_initializer->GetPosition(u_vec[i], NWU));
//    }
//
//    // Deriving the interpolation BSpline
//    std::vector<double> uk;
//    auto bspline_interp = bspline::internal::FrBSplineTools<2>::BSplineInterpFromPoints<3>(points, uk);
//
//
//    // *
//    // Converting the BSpline curve into Chrono counterpart
//    // *
//
//    auto ctrl_points = bspline_interp->GetCtrlPoints();
//    std::vector<chrono::ChVector<double>> ch_points;
//    ch_points.reserve(nb_ctrl_points);
//    for (int i = 0; i < nb_ctrl_points; i++) {
//      ch_points.push_back(internal::Vector3dToChVector(ctrl_points[i]));
//    }
//
//    auto knots = bspline_interp->GetKnotVector();
//    chrono::ChVectorDynamic<double> ch_knots;
//    ch_knots.Resize(knots.size());
//    for (int i = 0; i < knots.size(); i++) {
//      ch_knots(i) = knots[i];
//    }
//
//    auto ch_bspline = chrono::geometry::ChLineBspline(3, ch_points, &ch_knots);
//
//    // Building the beam using the Chrono tool
//    chrono::fea::ChBuilderBeamIGA builder;
//
//    builder.BuildBeam(m_chronoCable,
//                      m_section,
//                      ch_bspline,
//                      {0., 0., 1.});
//
//  }
//
//  Force FrFEACable::GetTension(const double &s, FRAME_CONVENTION fc) const {
//
//    assert(0. <= s && s <= GetUnstretchedLength());
//
//    double stmp = s;
//
//    if (s > GetUnstretchedLength()) stmp = GetUnstretchedLength(); // FIXME: ne devrait pas etre possible
//
//    double ds = GetUnstretchedLength() / GetNumberOfElements();
//    double a = stmp / ds;
//    auto index = int(floor(a));
//    double eta = 2. * (a - index) - 1.;
//
//    if (stmp == GetUnstretchedLength()) {
//      index = GetNumberOfElements() - 1;
//      eta = 1;
//    }
////        Force Tension;
//    auto Tension = m_chronoCable->GetTension(index, eta);
//
//    if (IsNED(fc)) internal::SwapFrameConvention(Tension);
//
//    return Tension;
//
//  }
//
//  Direction FrFEACable::GetTangent(const double &s, FRAME_CONVENTION fc) const {
//    return GetTension(s, fc).normalized();
//  }
//
//  Position FrFEACable::GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const {
//
//    assert(s <= GetUnstretchedLength());
//
//    double ds = GetUnstretchedLength() / GetNumberOfElements();
//    double a = s / ds;
//    auto index = int(floor(a));
//    double eta = 2. * (a - index) - 1.;
//
//    if (s == GetUnstretchedLength()) {
//      index = GetNumberOfElements() - 1;
//      eta = 1;
//    }
//
//    auto Pos = m_chronoCable->GetNodePositionInWorld(index, eta);
//
//    if (IsNED(fc)) internal::SwapFrameConvention(Pos);
//
//    return Pos;
//
//  }
//
////    void FrFEACable::AddFields() {
//////        if (IsLogged()) {
//////
//////            // Add the fields to be logged here
//////            m_message->AddField<double>("time", "s", "Current time of the simulation",
//////                                        [this]() { return m_system->GetTime(); });
//////
//////            m_message->AddField<double>("StrainedLength", "m", "Strained length of the catenary line",
//////                                        [this]() { return GetStrainedLength(); });
//////
//////            m_message->AddField<Eigen::Matrix<double, 3, 1>>
//////                    ("StartingNodeTension","N", fmt::format("Starting node tension in world reference frame in {}",GetLogFrameConvention()),
//////                     [this]() {return GetTension(0.,GetLogFrameConvention());});
//////
//////            m_message->AddField<Eigen::Matrix<double, 3, 1>>
//////                    ("EndingNodeTension","N", fmt::format("Ending node tension in world reference frame in {}",GetLogFrameConvention()),
//////                     [this]() { Eigen::Matrix<double, 3, 1> temp = -GetTension(GetUnstrainedLength(), GetLogFrameConvention());
//////                        return temp;});
//////
//////            //TODO : logger la position de la ligne pour un ensemble d'abscisses curvilignes?
//////
//////        }
////    }
//
//  void FrFEACable::SetStartingHingeType(FrFEACable::BOUNDARY_CONSTRAINT_TYPE type) {
//    m_startingHingeType = type;
//  }
//
//  FrFEACable::BOUNDARY_CONSTRAINT_TYPE FrFEACable::GetStartingHingeType() const {
//    return m_startingHingeType;
//  }
//
//  void FrFEACable::SetEndingHingeType(FrFEACable::BOUNDARY_CONSTRAINT_TYPE type) {
//    m_endingHingeType = type;
//  }
//
//  FrFEACable::BOUNDARY_CONSTRAINT_TYPE FrFEACable::GetEndingHingeType() const {
//    return m_endingHingeType;
//  }
//
//  double FrFEACable::GetStaticResidual() {
//
//    double residual = 0;
//
//    for (auto &node : m_chronoCable->GetNodes()) {
//      residual += dynamic_cast<chrono::fea::ChNodeFEAxyzrot *>(node.get())->GetPos_dt().Length();
//    }
//
//    return residual;
//  }
//
//  void FrFEACable::Relax() {
//
//    m_chronoCable->SetNoSpeedNoAcceleration();
//
//  }
//
//  std::shared_ptr<FrFEACable>
//  make_dynamic_cable(const std::string &name,
//                     const std::shared_ptr<FrNode> &startingNode,
//                     const std::shared_ptr<FrNode> &endingNode,
//                     const std::shared_ptr<FrCableProperties> &properties,
//                     double unstrainedLength,
//                     double rayleighDamping,
//                     unsigned int nbElements) {
//
//    auto Cable = std::make_shared<FrFEACable>(name,
//                                              startingNode,
//                                              endingNode,
//                                              properties,
//                                              unstrainedLength,
//                                              rayleighDamping,
//                                              nbElements);
//
//    startingNode->GetBody()->GetSystem()->Add(Cable);
//    return Cable;
//
//  }


} // end namespace frydom
