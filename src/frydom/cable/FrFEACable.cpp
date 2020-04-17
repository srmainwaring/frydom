//
// Created by lletourn on 05/03/19.
//

#include <chrono/fea/ChBeamSection.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/fea/ChNodeFEAxyzrot.h>
//#include <chrono/fea/ChElementBeamEuler.h>
#include <chrono/fea/ChVisualizationFEAmesh.h>
#include <chrono/fea/ChContactSurfaceNodeCloud.h>
#include <chrono/fea/ChContactSurfaceMesh.h>
#include <chrono/physics/ChLoadContainer.h>

#include "FrFEACable.h"

#include "FrFEACableLoads.h"
#include "FrCableProperties.h"

//#include "frydom/core/FrOffshoreSystem.h"
//#include "frydom/cable/FrCatenaryLine.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"

#include "frydom/logging/FrTypeNames.h"

#include "FrCableShapeInitializer.h"

namespace frydom {

  namespace internal {

    FrFEACableBase::FrFEACableBase(FrFEACable *cable) : chrono::fea::ChMesh(), m_frydomCable(cable) {

      m_startingHinge = std::make_shared<chrono::ChLinkMateGeneric>();
      m_endingHinge = std::make_shared<chrono::ChLinkMateGeneric>();
      m_section = std::make_shared<chrono::fea::ChBeamSectionAdvanced>();

    }

    void FrFEACableBase::InitializeSection() {

      m_section->SetAsCircularSection(m_frydomCable->GetProperties()->GetDiameter());
      m_section->SetBeamRaleyghDamping(m_frydomCable->GetRayleighDamping());
      m_section->SetDensity(m_frydomCable->GetProperties()->GetDensity());
      m_section->SetYoungModulus(m_frydomCable->GetProperties()->GetYoungModulus());
    }

    void FrFEACableBase::InitializeContact() {

      auto surface_material = std::make_shared<chrono::ChMaterialSurfaceSMC>();
      surface_material->SetYoungModulus(2e12f);
      surface_material->SetFriction(0.3f);
      surface_material->SetRestitution(0.0f);
      surface_material->SetAdhesion(0);
      surface_material->SetKn(2e12);
      surface_material->SetGn(1e6);


//      auto contact_surface = std::make_shared<chrono::fea::ChContactSurfaceMesh>();
//      AddContactSurface(contact_surface);
//      contact_surface->AddFacesFromBoundary(0.008);
//      contact_surface->SetMaterialSurface(surface_material);


      auto contact_surface = std::make_shared<chrono::fea::ChContactSurfaceNodeCloud>();
      AddContactSurface(contact_surface);
      contact_surface->AddAllNodes(m_frydomCable->GetProperties()->GetRadius());
      contact_surface->SetMaterialSurface(surface_material);

    }

    void FrFEACableBase::InitializeHydrodynamicLoads() {

      // Loads must be added to a load container
      auto load_container = std::make_shared<chrono::ChLoadContainer>();
      GetSystem()->Add(load_container);

      for (const auto& element : GetElements()) {
        auto loadable = std::dynamic_pointer_cast<internal::FrElementBeamEuler>(element);

        // Adding buoyancy
        auto load = std::make_shared<FrBuoyancyLoad>(m_frydomCable, loadable);
        load_container->Add(load);


      }



//      auto buoyancy_load = std::make_shared<FrBuoyancyLoader>(m_frydomCable->GetSystem());

      // Creer les autres loads !!


    }

    void FrFEACableBase::InitializeLinks() {

      // Starting hinge
      auto starting_body = m_frydomCable->GetStartingNode()->GetBody()->m_chronoBody;
      auto ChronoFrame = internal::FrFrame2ChFrame(m_frydomCable->GetStartingNode()->GetFrameInBody());

      m_startingHinge->Initialize(m_starting_node_fea, starting_body, true, chrono::ChFrame<double>(), ChronoFrame);

      // Ending hinge
      auto ending_body = m_frydomCable->GetEndingNode()->GetBody()->m_chronoBody;

      ChronoFrame = internal::FrFrame2ChFrame(m_frydomCable->GetEndingNode()->GetFrameInBody());
      FrFrame feaFrame;
      feaFrame.RotZ_RADIANS(MU_PI, NWU, false); // ending_node_fea comes from the opposite direction

      m_endingHinge->Initialize(m_ending_node_fea, ending_body, true, internal::FrFrame2ChFrame(feaFrame),
                                ChronoFrame);

      // Define the constraints on the hinges.
      HingesConstraints();

    }

    void FrFEACableBase::HingesConstraints() {

      switch (m_frydomCable->GetStartingHingeType()) {
        case FrFEACable::NONE:
          m_startingHinge->SetConstrainedCoords(false, false, false, false, false, false);
          break;
        case FrFEACable::SPHERICAL:
          m_startingHinge->SetConstrainedCoords(true, true, true, false, false, false);
          break;
        case FrFEACable::CONSTRAINED:
          m_startingHinge->SetConstrainedCoords(true, true, true, true, true, true);
          break;
      }

      switch (m_frydomCable->GetEndingHingeType()) {
        case FrFEACable::NONE:
          m_endingHinge->SetConstrainedCoords(false, false, false, false, false, false);
          break;
        case FrFEACable::SPHERICAL:
          m_endingHinge->SetConstrainedCoords(true, true, true, false, false, false);
          break;
        case FrFEACable::CONSTRAINED:
          m_endingHinge->SetConstrainedCoords(true, true, true, true, true, true);
          break;
      }

    }

    void FrFEACableBase::GenerateAssets() {
      // Assets for the cable visualisation
      if (m_drawCableElements) {
        auto elements_assets = std::make_shared<chrono::fea::ChVisualizationFEAmesh>(*this);
        elements_assets->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_TX);
        elements_assets->SetColorscaleMinMax(-m_frydomCable->GetBreakingTension(),
                                             m_frydomCable->GetBreakingTension()); //1799620
//                elements_assets->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_ANCF_BEAM_AX);
//                elements_assets->SetColorscaleMinMax(-0.4, 0.4);
        elements_assets->SetSmoothFaces(true);
        elements_assets->SetWireframe(false);
        m_section->SetDrawCircularRadius(m_frydomCable->GetDrawElementRadius());
        ChMesh::AddAsset(elements_assets);
      }

      // Assets for the nodes
      if (m_drawCableNodes) {
        auto node_assets = std::make_shared<chrono::fea::ChVisualizationFEAmesh>(*this);
        node_assets->SetFEMglyphType(
            chrono::fea::ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS); // E_GLYPH_NODE_CSYS
        node_assets->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_NONE);
        node_assets->SetSymbolsThickness(m_frydomCable->GetDrawNodeSize());
        node_assets->SetSymbolsScale(0.01);
        node_assets->SetZbufferHide(false);
        ChMesh::AddAsset(node_assets);
      }

    }

    void FrFEACableBase::Initialize() {

      if (m_starting_node_fea == nullptr) {

        InitializeSection();

//        // TODO: avoir un constructeur pour juste specifier les parametres du cable, pas les frontieres  -->  degager le constructeur par defaut
        Position distanceBetweenNodes = (m_frydomCable->GetEndingNode()->GetPositionInWorld(NWU) -
                                         m_frydomCable->GetStartingNode()->GetPositionInWorld(NWU));

        bool is_taut = distanceBetweenNodes.norm() >= m_frydomCable->GetUnstretchedLength();
        // TODO: voir si on a besoin de mitiger le fait que le cable est tendu ou pas directement dans cette methode

        auto shape_initializer = FrCableShapeInitializer::Create(m_frydomCable, m_frydomCable->GetSystem()->GetEnvironment());

        double s = 0.;
        double ds = m_frydomCable->GetUnstretchedLength() / m_frydomCable->GetNumberOfElements();

        // Compute the normal to the plan containing the cable
        auto AB = internal::Vector3dToChVector(m_frydomCable->GetEndingNode()->GetPositionInWorld(NWU) -
                                               m_frydomCable->GetStartingNode()->GetPositionInWorld(NWU));
        AB.Normalize(); // FIXME: polutot travailler avec les objets frydom en premiere instance et les convertir en chrono au dernier moment...

        // Init with the starting node
        auto ChronoFrame = internal::FrFrame2ChFrame(m_frydomCable->GetStartingNode()->GetFrameInWorld());

        chrono::ChVector<double> e1, e2, e3;
        chrono::ChMatrix33<double> RotMat;

        if (!is_taut) {
          e1 = internal::Vector3dToChVector(shape_initializer->GetTangent(0., NWU));
          e3 = e1.Cross(AB);
          e3.Normalize();
          e2 = e3.Cross(e1);
          e2.Normalize();

          RotMat.Set_A_axis(e1, e2, e3);

          ChronoFrame.SetPos(
              internal::Vector3dToChVector(m_frydomCable->GetStartingNode()->GetPositionInWorld(NWU)));
          ChronoFrame.SetRot(RotMat);
        }

        auto nodeA = std::make_shared<chrono::fea::ChNodeFEAxyzrot>(ChronoFrame);
        m_starting_node_fea = nodeA;

        // Add the node to the ChMesh
        AddNode(m_starting_node_fea);

        // Creating the specified number of Cable elements
        for (uint i = 1; i <= m_frydomCable->GetNumberOfElements(); ++i) {
          s += ds;

          // Get the position and direction of the line for the curvilinear coord s
          if (is_taut) {
            ChronoFrame.SetPos(ChronoFrame.GetPos() + ds * AB);
          } else {
            ChronoFrame.SetPos(internal::Vector3dToChVector(shape_initializer->GetPosition(s, NWU)));
            e1 = internal::Vector3dToChVector(shape_initializer->GetTangent(s, NWU));
//            e1.Normalize();
            e2 = e3.Cross(e1);
            e2.Normalize();
            RotMat.Set_A_axis(e1, e2, e3);
            ChronoFrame.SetRot(RotMat);
          }

          // Create a node and add it to the ChMesh
          auto nodeB = std::make_shared<chrono::fea::ChNodeFEAxyzrot>(ChronoFrame);
          AddNode(nodeB);

          // Create a cable element between the nodes A and B, and add it to the ChMesh
          auto element = std::make_shared<internal::FrElementBeamEuler>();
          element->SetNodes(nodeA, nodeB);
          element->SetSection(m_section);
          AddElement(element);

          //
          nodeA = nodeB;

        }

        // FIXME: c'es etrange ce qui se passe ici, dans ce cas, pourquoi ne pas declarer nodeB avant la boucle ??
        // Add the ending node to the ChMesh
        m_ending_node_fea = nodeA; // nodeB is destroyed after the loop
//                AddNode(m_ending_node_fea);

        // Generate constraints between boundaries and bodies
        InitializeLinks();

        if (m_frydomCable->GetBreakingTension() == 0.) {

          if (is_taut) {
            double tensionMax = (distanceBetweenNodes.norm() - m_frydomCable->GetUnstretchedLength()) *
                m_frydomCable->GetProperties()->GetEA() / m_frydomCable->GetUnstretchedLength();
            m_frydomCable->SetBreakingTension(1.2 * tensionMax);
          } else {
            // TODO: remettre en place le mecanisme ...
//            m_frydomCable->SetBreakingTension(1.2 * catenaryLine->GetMaxTension());
          }

        }

        // Set reference position of nodes as current position, for all nodes.
        Relax();

        InitializeHydrodynamicLoads();

        InitializeContact();

        // Generate assets for the cable
        GenerateAssets();

      }


      // Absolutely necessary for finite elements !
      SetupInitial();

    }

    void FrFEACableBase::Update(double time, bool update_assets) {

//      UpdateForces(time);

      chrono::fea::ChMesh::Update(time, update_assets);
      m_frydomCable->Update(time);

    }

//    void FrFEACableBase::UpdateForces(double time) {
//
//      // FIXME: ce n'est peut-etre pas ici qu'il faut faire qqch...
//
//
//
//
//    }

    Position FrFEACableBase::GetNodePositionInWorld(int index, double eta) {

      chrono::ChVector<double> Pos;
      chrono::ChQuaternion<double> Rot;

      dynamic_cast<internal::FrElementBeamEuler *>(GetElement(index).get())->EvaluateSectionFrame(eta, Pos, Rot);

      return internal::ChVectorToVector3d<Position>(Pos);
    }

    Force FrFEACableBase::GetTension(int index, double eta) {

      chrono::ChVector<double> Tension, Torque;

      auto element = dynamic_cast<internal::FrElementBeamEuler *>(GetElement(index).get());

      element->EvaluateSectionForceTorque(eta, Tension, Torque);

      auto dir = element->GetNodeB()->GetPos() - element->GetNodeA()->GetPos();
      dir.Normalize();

      // only the traction is kept (no bending, etc.)
      return internal::ChVectorToVector3d<Force>(dir * Tension.x());

    }
  }

  FrFEACable::FrFEACable(const std::string &name,
                         const std::shared_ptr<frydom::FrNode> &startingNode,
                         const std::shared_ptr<frydom::FrNode> &endingNode,
                         const std::shared_ptr<FrCableProperties> &properties,
                         double unstrainedLength,
                         double rayleighDamping,
                         unsigned int nbElements) :
      FrLoggable(name, TypeToString(this), startingNode->GetBody()->GetSystem()),
      FrFEAMesh(),
      FrCableBase(startingNode, endingNode, properties, unstrainedLength),
      m_rayleighDamping(rayleighDamping),
      m_nbElements(nbElements) {

    m_chronoCable = std::make_shared<internal::FrFEACableBase>(this);
    LogThis(true);

  }


  void FrFEACable::SetRayleighDamping(double damping) {
    m_rayleighDamping = damping;
  }

  double FrFEACable::GetRayleighDamping() const {
    return m_rayleighDamping;
  }

  void FrFEACable::SetNumberOfElements(unsigned int nbElements) {
    m_nbElements = nbElements;
  }

  unsigned int FrFEACable::GetNumberOfElements() const {
    return m_nbElements;
  }

  void FrFEACable::SetTargetElementLength(double elementLength) {
    assert(elementLength > 0. && elementLength < GetUnstretchedLength());
    m_nbElements = static_cast<unsigned int>(int(floor(GetUnstretchedLength() / elementLength)));
  }

  void FrFEACable::DefineLogMessages() {

    auto msg = NewMessage("State", "State message");

    msg->AddField<double>("time", "s", "Current time of the simulation",
                          [this]() { return GetSystem()->GetTime(); });

    msg->AddField<double>("StrainedLength", "m", "Strained Length of the dynamic cable",
                          [this]() { return GetStrainedLength(); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("StartingNodeTension", "N", fmt::format("Starting node tension in world reference frame in {}", GetLogFC()),
         [this]() { return GetTension(0., GetLogFC()); });

    msg->AddField<Eigen::Matrix<double, 3, 1>>
        ("EndingNodeTension", "N", fmt::format("Ending node tension in world reference frame in {}", GetLogFC()),
         [this]() { return -GetTension(GetUnstretchedLength(), GetLogFC()); });

  }

  void FrFEACable::SetBreakingTension(double tension) {
    m_maxTension = tension;
  }

  double FrFEACable::GetBreakingTension() const {
    return m_maxTension;
  }

  void FrFEACable::SetDrawElementRadius(double radius) {
    m_drawCableElementRadius = radius;
  }

  double FrFEACable::GetDrawElementRadius() {
    return m_drawCableElementRadius;
  }

  void FrFEACable::SetDrawNodeSize(double size) {
    m_drawCableNodeSize = size;
  }

  double FrFEACable::GetDrawNodeSize() const {
    return m_drawCableElementRadius;
  }

  void FrFEACable::Initialize() {

    m_chronoCable->Initialize();

    GetTension(0., NWU);
  }

  Force FrFEACable::GetTension(const double &s, FRAME_CONVENTION fc) const {

    assert(0. <= s && s <= GetUnstretchedLength());

    double stmp = s;

    if (s > GetUnstretchedLength()) stmp = GetUnstretchedLength(); // FIXME: ne devrait pas etre possible

    double ds = GetUnstretchedLength() / GetNumberOfElements();
    double a = stmp / ds;
    auto index = int(floor(a));
    double eta = 2. * (a - index) - 1.;

    if (stmp == GetUnstretchedLength()) {
      index = GetNumberOfElements() - 1;
      eta = 1;
    }
//        Force Tension;
    auto Tension = m_chronoCable->GetTension(index, eta);

    if (IsNED(fc)) internal::SwapFrameConvention(Tension);

    return Tension;

  }

  Direction FrFEACable::GetTangent(const double &s, FRAME_CONVENTION fc) const {
    return GetTension(s, fc).normalized();
  }

  Position FrFEACable::GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const {

    assert(s <= GetUnstretchedLength());

    double ds = GetUnstretchedLength() / GetNumberOfElements();
    double a = s / ds;
    auto index = int(floor(a));
    double eta = 2. * (a - index) - 1.;

    if (s == GetUnstretchedLength()) {
      index = GetNumberOfElements() - 1;
      eta = 1;
    }

    auto Pos = m_chronoCable->GetNodePositionInWorld(index, eta);

    if (IsNED(fc)) internal::SwapFrameConvention(Pos);

    return Pos;

  }

//    void FrFEACable::AddFields() {
////        if (IsLogged()) {
////
////            // Add the fields to be logged here
////            m_message->AddField<double>("time", "s", "Current time of the simulation",
////                                        [this]() { return m_system->GetTime(); });
////
////            m_message->AddField<double>("StrainedLength", "m", "Strained length of the catenary line",
////                                        [this]() { return GetStrainedLength(); });
////
////            m_message->AddField<Eigen::Matrix<double, 3, 1>>
////                    ("StartingNodeTension","N", fmt::format("Starting node tension in world reference frame in {}",GetLogFrameConvention()),
////                     [this]() {return GetTension(0.,GetLogFrameConvention());});
////
////            m_message->AddField<Eigen::Matrix<double, 3, 1>>
////                    ("EndingNodeTension","N", fmt::format("Ending node tension in world reference frame in {}",GetLogFrameConvention()),
////                     [this]() { Eigen::Matrix<double, 3, 1> temp = -GetTension(GetUnstrainedLength(), GetLogFrameConvention());
////                        return temp;});
////
////            //TODO : logger la position de la ligne pour un ensemble d'abscisses curvilignes?
////
////        }
//    }

  void FrFEACable::SetStartingHingeType(FrFEACable::HingeType type) {
    m_startingHingeType = type;
  }

  FrFEACable::HingeType FrFEACable::GetStartingHingeType() const {
    return m_startingHingeType;
  }

  void FrFEACable::SetEndingHingeType(FrFEACable::HingeType type) {
    m_endingHingeType = type;
  }

  FrFEACable::HingeType FrFEACable::GetEndingHingeType() const {
    return m_endingHingeType;
  }

  double FrFEACable::GetStaticResidual() {

    double residual = 0;

    for (auto &node : m_chronoCable->GetNodes()) {
      residual += dynamic_cast<chrono::fea::ChNodeFEAxyzrot *>(node.get())->GetPos_dt().Length();
    }

    return residual;
  }

  void FrFEACable::Relax() {

    m_chronoCable->SetNoSpeedNoAcceleration();

  }

  std::shared_ptr<FrFEACable>
  make_dynamic_cable(const std::string &name,
                     const std::shared_ptr<FrNode> &startingNode,
                     const std::shared_ptr<FrNode> &endingNode,
                     const std::shared_ptr<FrCableProperties> &properties,
                     double unstrainedLength,
                     double rayleighDamping,
                     unsigned int nbElements) {

    auto Cable = std::make_shared<FrFEACable>(name,
                                              startingNode,
                                              endingNode,
                                              properties,
                                              unstrainedLength,
                                              rayleighDamping,
                                              nbElements);

    startingNode->GetBody()->GetSystem()->Add(Cable);
    return Cable;

  }

} // end namespace frydom
