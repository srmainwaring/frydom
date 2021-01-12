//
// Created by camille on 27/03/2020.
//

#include "FrBarElement.h"

namespace frydom {

  // --------------------------------------------------
  // Bar element
  // --------------------------------------------------

  FrBarElementBase::FrBarElementBase()
  : m_node(), m_force(), m_torque(), m_is_immerged(false) {}

  Force FrBarElementBase::GetForce() const {
    return m_force;
  }

  Torque FrBarElementBase::GetTorque() const {
    return m_torque;
  }

  bool FrBarElementBase::IsImmerged() const {
    return m_is_immerged;
  }

  std::shared_ptr<FrNode>& FrBarElementBase::GetNode() {
    return m_node;
  }

  // --------------------------------------------------
  // Single bar element
  // --------------------------------------------------

  FrSingleBarElement::FrSingleBarElement(FrBody* body, Position posA, Position posB, double radius)
   : m_radius(radius) {

    SetFrame(body, posA, posB);

    m_node_start = std::make_shared<FrNode>("", body);
    m_node_start->SetPositionInBody(posA, NWU);

    m_node_end = std::make_shared<FrNode>("", body);
    m_node_end->SetPositionInBody(posB, NWU);

    m_length = (posB - posA).norm();
    m_volume = MU_PI * m_radius * m_radius * m_length;

  }

  void FrSingleBarElement::SetFrame(FrBody* body, Position& posA, Position& posB) {

    Position center = 0.5 * (posA + posB);

    Direction e3 = posB - posA;
    e3.normalize();
    Direction e1 = Direction(0., 0., 1.).cross(e3);

    if (std::abs(e1.norm()) > FLT_EPSILON) {
      e1.normalize();
    } else {
      e1 = Direction(1., 0., 0.);
    }

    Direction e2 = e3.cross(e1);
    e2.normalize();

    m_node = std::make_shared<FrNode>("", body);
    m_node->SetFrameInBody(FrFrame(center, FrRotation(e1, e2, e3, NWU), NWU));
  }

  void FrSingleBarElement::Update(double time) {

    auto freeSurface = m_node->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface();

    auto pos = m_node->GetPositionInWorld(NWU);
    auto waveElevation = freeSurface->GetElevation(pos.x(), pos.y(), NWU);

    if (pos.z() - m_length > waveElevation) {
      NullifyForceTorque();

    } else if (pos.z() + m_length < waveElevation) {
      SetForceTorque();

    } else {

      auto posA = m_node_start->GetPositionInWorld(NWU);
      auto waveElevationA = freeSurface->GetElevation(posA.x(), posA.y(), NWU);
      double wA = posA.z() - waveElevationA;

      auto posB = m_node_end->GetPositionInWorld(NWU);
      auto waveElevationB = freeSurface->GetElevation(posB.x(), posB.y(), NWU);
      double wB = posB.z() - waveElevationB;

      if (wA < DBL_EPSILON) {
        if (wB < DBL_EPSILON) {
          SetForceTorque();
        } else {
          SetForceTorque(posA, wA, posB, wB);
        }
      } else if (wB < DBL_EPSILON) {
        SetForceTorque(posB, wB, posA, wA);
      } else {
        NullifyForceTorque();
      }
    }
  }

  void FrSingleBarElement::Initialize() {

    auto system = m_node->GetSystem();

    auto gravity = system->GetGravityAcceleration();
    auto water_density = system->GetEnvironment()->GetOcean()->GetDensity();

    m_buoyancy = gravity * water_density * m_volume;
    m_force = Force(0., 0., m_buoyancy);
  }

  void FrSingleBarElement::NullifyForceTorque() {
    m_is_immerged = false;
    m_force.SetNull();
    m_torque.SetNull();
  }

  void FrSingleBarElement::SetForceTorque() {

    m_is_immerged = true;
    m_force = Force(0., 0., m_buoyancy);
    auto body = m_node->GetBody();
    auto relPos = m_node->GetPositionInWorld(NWU) - body->GetCOGPositionInWorld(NWU);
    m_torque = relPos.cross(m_force);

  }

  void FrSingleBarElement::SetForceTorque(const Position& posA, const double wA,
                                          const Position& posB, const double wB) {

    m_is_immerged = true;

    auto ratio = 1. / (1. + std::abs(wB / wA));

    Direction vect = posB - posA;
    Position center = posA + 0.5 * ratio * vect;
    auto body = m_node->GetBody();
    auto relPos = center - body->GetCOGPositionInWorld(NWU);

    m_force = Force(0., 0., m_buoyancy * ratio);
    m_torque = relPos.cross(m_force);
  }

  // --------------------------------------------------
  // Composite bar element
  // --------------------------------------------------

  FrCompositeBarElement::FrCompositeBarElement(FrBody* body) : m_cfactor(0.) {
    m_node = std::make_shared<FrNode>("", body);
  }

  void FrCompositeBarElement::AddElement(FrBarElementBase* element) {
    m_elements.push_back(std::unique_ptr<FrBarElementBase>(element));
  }

  void FrCompositeBarElement::AddElement(Position posA, Position posB, double radius, unsigned int n) {
    Direction dV = (posB - posA) / n;
    Position pos;
    for (unsigned int i=0; i<n; ++i) {
      pos = posA + dV*i;
      m_elements.push_back(std::make_unique<FrSingleBarElement>(m_node->GetBody(), pos, pos+dV, radius));
    }
  }

  void FrCompositeBarElement::Initialize() {
    for (auto& element : m_elements) {
      element->Initialize();
    }
  }

  void FrCompositeBarElement::Update(double time) {
    m_force.SetNull();
    m_torque.SetNull();
    for (auto& element : m_elements) {
      element->Update(time);
      if (element->IsImmerged()) {
        m_force += element->GetForce();
        m_torque += element->GetTorque();
      }
    }

    m_force *= (1 + m_cfactor);
    m_torque *= (1 + m_cfactor);
  }

  void FrCompositeBarElement::SetCorrectionFactor(double cfactor) {
    m_cfactor = cfactor;
  }

  // ----------------------------------------------------
  // Makers
  // ----------------------------------------------------

  std::shared_ptr<FrCompositeBarElement> make_bar_element(const std::shared_ptr<FrBody>& body) {
    return std::make_shared<FrCompositeBarElement>(body.get());
  }

  std::shared_ptr<FrCompositeBarElement> make_bar_element(const std::shared_ptr<FrBody>& body,
                                                          const std::string& filename) {
    auto model = std::make_shared<FrCompositeBarElement>(body.get());

    std::ifstream ifs(filename);
    auto json_obj = json::parse(ifs);

    auto nodes = json_obj["Nodes"];

    std::map<int, std::vector<double>> node_map;
    for (auto node: nodes) {
      node_map.emplace(std::make_pair(node["id"].get<int>(), node["position"].get<std::vector<double>>()));
    }

    auto elements = json_obj["Elements"];

    std::vector<double> position;
    double diameter;

    for (const auto& element: elements) {
      position = node_map.at(element["id1"].get<int>());
      Position posA = {position[0], position[1], position[2]};
      position = node_map.at(element["id2"].get<int>());
      Position posB = {position[0], position[1], position[2]};
      diameter = element["diameter"].get<double>();
      Direction dv = (posB - posA);
      model->AddElement(posA, posB, 0.5*diameter);
    }

    return model;

  }

} // end namespace frydom
