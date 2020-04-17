// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#include "FrMorisonModel.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/environment/FrEnvironmentInc.h"
#include "frydom/hydrodynamics/morison/FrMorisonModelBase.h"


namespace frydom {


  std::shared_ptr<FrMorisonCompositeElement> make_morison_model(const std::string& name,
                                                                const std::shared_ptr<FrBody> &body,
                                                                bool extendedModel) {
    auto model = std::make_shared<FrMorisonCompositeElement>(name, body.get(), extendedModel);
    body->GetSystem()->Add(model);
    return model;
  }


  std::shared_ptr<FrMorisonCompositeElement> make_morison_model(const std::string& name,
                                                                const std::shared_ptr<FrBody> &body,
                                                                const std::string &filename,
                                                                bool extendedModel) {
    //TODO ; extend for Cf, double valued Cd and Ca, etc.
    auto model = std::make_shared<FrMorisonCompositeElement>(name, body.get(), extendedModel);

    std::ifstream ifs(filename);
    auto json_obj = json::parse(ifs);

    auto nodes = json_obj["Nodes"];

    std::map<int, std::vector<double>> node_map;
    for (auto node : nodes) {
      node_map.emplace(std::make_pair(node["id"].get<int>(), node["position"].get<std::vector<double>>()));
    }

    auto elements = json_obj["Elements"];

    std::vector<double> position;
    double Cd, Cm, diameter;

    for (const auto &element : elements) {
      position = node_map.at(element["id1"].get<int>());
      Position PosA = {position[0],position[1],position[2]};
      position = node_map.at(element["id2"].get<int>());
      Position PosB = {position[0],position[1],position[2]};
      diameter = element["diameter"].get<double>();
      Cd = element["cd"].get<double>();
      Cm = element["cm"].get<double>();
      Direction dv = (PosB - PosA);
      model->AddElement(PosA, PosB, diameter, MorisonCoeff(Cm-1.), MorisonCoeff(Cd), 0.);
    }

    body->GetSystem()->Add(model);

    return model;
  }

  // -----------------------------------------------------------------
  // MORISON MODEL
  // -----------------------------------------------------------------

  FrMorisonElement::FrMorisonElement() : m_node(nullptr), m_force(), m_torque(),
  m_includeCurrent(false), m_extendedModel(false), m_isImmerged(false),
  m_force_added_mass(), m_torque_added_mass(),
  m_AMInFrame(6, 6), m_AMInWorld(6, 6), m_AMInBody(6, 6) {}

  void FrMorisonElement::SetFrame(FrBody *body, Position posA, Position posB, Direction vect) {

    Position position = 0.5 * (posA + posB);

    Direction e3 = posB - posA;
    e3.normalize();
    Direction e1 = vect.cross(e3);

    if (std::abs(e1.norm()) > FLT_EPSILON) {
      e1.normalize();
    } else {
      e1 = Direction(1., 0., 0.);
    }

    Direction e2 = e3.cross(e1);
    e2.normalize();

//        m_node = std::make_shared<FrNode>(body, position, FrRotation(e1, e2, e3, NWU));
    m_node = std::make_shared<FrNode>("",
                                      body);  // TODO : doit etre gere par la classe de base !! FIXME : comment nommer tous les noeuds pour pas avoir de doublon ???
    m_node->SetFrameInBody(FrFrame(position, FrRotation(e1, e2, e3, NWU), NWU));
  }

  void FrMorisonElement::SetFrame(FrBody *body, const FrFrame &frame) {
    m_node = std::make_shared<FrNode>("", body);
    m_node->SetFrameInBody(frame);
  }

  Force FrMorisonElement::GetForceInWorld(FRAME_CONVENTION fc) const {
    auto force = m_force;
    if (IsNED(fc)) internal::SwapFrameConvention(force);
    return force;
  }

  Torque FrMorisonElement::GetTorqueInBody() const {
    return m_torque;
  }

  Force FrMorisonElement::GetForceAddedMassInWorld(FRAME_CONVENTION fc) const {
    auto force = m_force_added_mass;
    if (IsNED(fc)) internal::SwapFrameConvention(force);
    return force;
  }

  Torque FrMorisonElement::GetTorqueAddedMassInWorld() const {
    return m_torque_added_mass;
  }

  FrFrame FrMorisonElement::GetFrame() const { return m_node->GetFrameInWorld(); }

  FrBody* FrMorisonElement::GetBody() const { return m_node->GetBody(); }

  const Eigen::Matrix<double, 6, 6>& FrMorisonElement::GetAMInFrame() {
    return m_AMInFrame;
  }

  const Eigen::Matrix<double, 6, 6>& FrMorisonElement::GetAMInBody() {
    return m_AMInBody;
  }

  const Eigen::Matrix<double, 6, 6>& FrMorisonElement::GetAMInWorld() {
    return m_AMInWorld;
  }

  // ---------------------------------------------------------------------
  // MORISON SINGLE ELEMENT
  // ---------------------------------------------------------------------

  FrMorisonSingleElement::FrMorisonSingleElement(FrBody *body) {
    m_node = std::make_shared<FrNode>("", body);
  }

  FrMorisonSingleElement::FrMorisonSingleElement(FrBody *body, Position posA, Position posB, double diameter,
                                                 MorisonCoeff ca, MorisonCoeff cd, double cf,
                                                 Direction perpendicular) {
    SetAddedMass(ca);
    SetDragCoeff(cd);
    SetFrictionCoeff(cf);

    //m_node = std::make_shared<FrNode>(body);
    SetFrame(body, posA, posB, perpendicular);

    SetDiameter(diameter);
    SetLength(posA, posB);
    SetVolume();
  }

  FrMorisonSingleElement::FrMorisonSingleElement(std::shared_ptr<FrNode> &nodeA,
                                                 std::shared_ptr<FrNode> &nodeB,
                                                 double diameter, MorisonCoeff ca, MorisonCoeff cd, double cf,
                                                 Direction perpendicular) {
    SetNodes(nodeA, nodeB);

    SetAddedMass(ca);
    SetDragCoeff(cd);
    SetFrictionCoeff(cf);

    //m_node = std::make_shared<FrNode>(nodeA->GetBody());
    SetFrame(nodeA->GetBody(), nodeA->GetNodePositionInBody(NWU), nodeB->GetNodePositionInBody(NWU), perpendicular);

    SetDiameter(diameter);
    SetLength(nodeA->GetPositionInWorld(NWU), nodeB->GetPositionInWorld(NWU));
    SetVolume();
  }

  FrMorisonSingleElement::FrMorisonSingleElement(FrBody *body, FrFrame frame, double diameter, double length,
                                                 MorisonCoeff ca, MorisonCoeff cd, double cf) {
    SetAddedMass(ca);
    SetDragCoeff(cd);
    SetFrictionCoeff(cf);

    SetFrame(body, frame);

    SetDiameter(diameter);
    SetLength(length);
    SetVolume();
  }


  void FrMorisonSingleElement::SetNodes(std::shared_ptr<FrNode> &nodeA, std::shared_ptr<FrNode> &nodeB) {
    m_nodeA = nodeA;
    m_nodeB = nodeB;
  }

  void FrMorisonSingleElement::SetNodes(FrBody *body, Position posA, Position posB) {
    m_nodeA = std::make_shared<FrNode>("", body);
    m_nodeA->SetPositionInBody(posA, NWU);
    m_nodeB = std::make_shared<FrNode>("", body);
    m_nodeB->SetPositionInBody(posB, NWU);
    SetLength(m_nodeA->GetPositionInWorld(NWU), m_nodeB->GetPositionInWorld(NWU));
  }

  void FrMorisonSingleElement::SetAddedMass(MorisonCoeff ca) {
    assert(ca.x >= -FLT_EPSILON or std::abs(ca.x) <= FLT_EPSILON);
    assert(ca.y >= -FLT_EPSILON or std::abs(ca.y) <= FLT_EPSILON);
    m_property.ca = ca;
  }

  void FrMorisonSingleElement::SetDragCoeff(MorisonCoeff cd) {
    assert(cd.x >= -FLT_EPSILON or std::abs(cd.x) <= FLT_EPSILON);
    assert(cd.y >= -FLT_EPSILON or std::abs(cd.y) <= FLT_EPSILON);
    m_property.cd = cd;
  }

  void FrMorisonSingleElement::SetFrictionCoeff(double cf) {
    assert(cf >= -FLT_EPSILON or std::abs(cf) <= FLT_EPSILON);
    m_property.cf = cf;
  }

  void FrMorisonSingleElement::SetDiameter(const double diameter) {
    assert(diameter >= -FLT_EPSILON or std::abs(diameter) <= FLT_EPSILON);
    m_property.diameter = diameter;
  }

  void FrMorisonSingleElement::SetLength(Position posA, Position posB) {
    m_property.length = (posB - posA).norm();
  }

  void FrMorisonSingleElement::SetVolume() {
    m_property.volume = MU_PI_4 * GetDiameter() * GetDiameter() * GetLength();
  }


  void FrMorisonSingleElement::SetAMInFrame() {

    m_AMInFrame.setZero();

    auto body = m_node->GetBody();
    auto rho = body->GetSystem()->GetEnvironment()->GetOcean()->GetDensity();

    auto ca = Vector3d<double>(m_property.ca.x, m_property.ca.y, 0.);

    Position posInBody = m_node->GetNodePositionInBody(NWU) - body->GetCOG(NWU);
    Position posInFrame = m_node->GetFrameInBody().ProjectVectorParentInFrame(posInBody, NWU);

    m_AMInFrame(0, 0) =  ca[0];
    m_AMInFrame(0, 4) =  ca[0] * posInFrame[2];
    m_AMInFrame(0, 5) = -ca[0] * posInFrame[1];
    m_AMInFrame(1, 1) =  ca[1];
    m_AMInFrame(1, 3) = -ca[1] * posInFrame[2];
    m_AMInFrame(1, 5) =  ca[1] * posInFrame[0];
    //m_AM(2, 2) =  ca[2]; // = 0
    //m_AM(2, 3) =  ca[2] * posInFrame[1];  // = 0
    //m_AM(2, 4) = -ca[2] * posInFrame[0]; // = 0

    m_AMInFrame(3, 1) = -ca[1] * posInFrame[2];
    m_AMInFrame(3, 3) =  ca[1] * posInFrame[2] * posInFrame[2];
    m_AMInFrame(3, 5) = -ca[1] * posInFrame[0] * posInFrame[2];
    m_AMInFrame(4, 0) =  ca[0] * posInFrame[2];
    m_AMInFrame(4, 4) =  ca[0] * posInFrame[2] * posInFrame[2];
    m_AMInFrame(4, 5) = -ca[0] * posInFrame[1] * posInFrame[2];
    m_AMInFrame(5, 0) = -ca[0] * posInFrame[1];
    m_AMInFrame(5, 1) =  ca[1] * posInFrame[0];
    m_AMInFrame(5, 3) = -ca[1] * posInFrame[0] * posInFrame[2];
    m_AMInFrame(5, 4) = -ca[0] * posInFrame[1] * posInFrame[2];
    m_AMInFrame(5, 5) =  ca[0] * posInFrame[1] * posInFrame[1] + ca[1] * posInFrame[0] * posInFrame[0];

    m_AMInFrame *= rho * GetVolume();
  }

  void FrMorisonSingleElement::SetAMInBody() {
    m_AMInBody.setZero();
    auto matrix = m_node->GetFrameInBody().GetRotation().GetRotationMatrix();
    m_AMInBody.block<3, 3>(0, 0) = matrix * m_AMInFrame.block<3, 3>(0, 0) * matrix.inverse();
    m_AMInBody.block<3, 3>(0, 3) = matrix * m_AMInFrame.block<3, 3>(0, 3) * matrix.inverse();
    m_AMInBody.block<3, 3>(3, 0) = matrix * m_AMInFrame.block<3, 3>(3, 0) * matrix.inverse();
    m_AMInBody.block<3, 3>(3, 3) = matrix * m_AMInFrame.block<3, 3>(3, 3) * matrix.inverse();
  }

  void FrMorisonSingleElement::SetAMInWorld() {
    m_AMInWorld.setZero();
    auto matrix = m_node->GetFrameInWorld().GetRotation().GetRotationMatrix();
    m_AMInWorld.block<3, 3>(0, 0) = matrix * m_AMInFrame.block<3, 3>(0, 0) * matrix.inverse();
    m_AMInWorld.block<3, 3>(0, 3) = matrix * m_AMInFrame.block<3, 3>(0, 3) * matrix.inverse();
    m_AMInWorld.block<3, 3>(3, 0) = matrix * m_AMInFrame.block<3, 3>(3, 0) * matrix.inverse();
    m_AMInWorld.block<3, 3>(3, 3) = matrix * m_AMInFrame.block<3, 3>(3, 3) * matrix.inverse();
  }

  void FrMorisonSingleElement::CheckImmersion() {

    auto pos = m_node->GetPositionInWorld(NWU);
    auto waveField = m_node->GetBody()->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetWaveField();
    auto waveElevation = waveField->GetElevation(pos.x(), pos.y(), NWU);

    m_isImmerged = pos.z() < waveElevation + DBL_EPSILON;

  }

  Velocity FrMorisonSingleElement::GetFlowVelocity() {

    Velocity velocity;
    Position worldPos = m_node->GetPositionInWorld(NWU);
    auto body = m_node->GetBody();

    auto waveField = body->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetWaveField();

    velocity = waveField->GetVelocity(worldPos, NWU);
    velocity -= m_node->GetVelocityInWorld(NWU);

    if (m_includeCurrent) {
      velocity += body->GetSystem()->GetEnvironment()->GetOcean()->GetCurrent()->GetFluxVelocityInWorld(worldPos, NWU);
    }

    //Velocity velocityBody = body->GetFrame().ProjectVectorParentInFrame(velocity, NWU);
    Velocity velocityNode = m_node->GetFrameInWorld().ProjectVectorParentInFrame(velocity, NWU);

    //##CC
    if (not m_isImmerged) {
      velocityNode = {0., 0., 0.};
    }
    //##CC

    return velocityNode;
  }

  Acceleration FrMorisonSingleElement::GetFlowAcceleration() {

    Acceleration acceleration;
    Position worldPos = m_node->GetPositionInWorld(NWU);
    auto body = m_node->GetBody();

    auto waveField = body->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetWaveField();

    acceleration = waveField->GetAcceleration(worldPos, NWU);
    //Acceleration accBody = body->GetFrame().ProjectVectorParentInFrame(acceleration, NWU);
    auto accNode = m_node->GetFrameInWorld().ProjectVectorParentInFrame(acceleration, NWU);

    //##CC
    if (not m_isImmerged) {
      accNode= {0., 0., 0.};
    }
    //##

    return accNode;
  }

  Acceleration FrMorisonSingleElement::GetNodeAcceleration() {
    Acceleration acceleration = m_node->GetAccelerationInWorld(NWU);
    Acceleration accBody = m_node->GetBody()->GetFrame().ProjectVectorParentInFrame(acceleration, NWU);
    return m_node->GetFrameInWorld().ProjectVectorParentInFrame(acceleration, NWU);
  }

  //
  // UPDATE
  //

  void FrMorisonSingleElement::Update(double time) {

    auto body = m_node->GetBody();
    auto rho = body->GetSystem()->GetEnvironment()->GetOcean()->GetDensity();

    // Nullify initial values of force and torque
    m_force_added_mass.SetNull();
    m_torque_added_mass.SetNull();

    // Check if the element is immerged
    CheckImmersion();

    // Flow Velocity part
    Velocity velocity = GetFlowVelocity();
    Velocity velocity_n = {velocity.x(), velocity.y(), 0.};
    double Vnorm_n = velocity_n.norm();

    Force localForce;
    Vector3d<double> Cd = {m_property.cd.x, m_property.cd.y, M_PI * m_property.cf};
    localForce = 0.5 * rho * m_property.diameter * m_property.length * Vnorm_n * velocity_n.cwiseProduct(Cd);

    // Flow acceleration part
    if (m_extendedModel and m_isImmerged) {
      Acceleration acceleration = GetFlowAcceleration();
      localForce.x() += rho * (m_property.ca.x + 1.) * GetVolume() * acceleration.x();
      localForce.y() += rho * (m_property.ca.y + 1.) * GetVolume() * acceleration.y();
    }

    // Project local force in world at COG
    m_force = m_node->GetFrameInWorld().ProjectVectorFrameInParent(localForce, NWU);

    // Part the added mass term due to the body's angular speed (in world ref frame)
    if (m_extendedModel and m_isImmerged) {

      Vector3d<double> ca = {m_property.ca.x, m_property.ca.y, 0.};

      AngularVelocity omegaInWorld = body->GetAngularVelocityInWorld(NWU);
      AngularVelocity omegaInFrame = m_node->GetFrameInWorld().ProjectVectorParentInFrame(omegaInWorld, NWU);

      Position relPosInWorld = m_node->GetPositionInWorld(NWU) - body->GetCOGPositionInWorld(NWU);
      Position relPosInBody = m_node->GetNodePositionInBody(NWU) - body->GetCOG(NWU);
      Position relPosInFrame = m_node->GetFrameInBody().ProjectVectorParentInFrame(relPosInBody, NWU);

      Force forceAngSpeedInFrame;
      forceAngSpeedInFrame = omegaInFrame.cross(omegaInFrame.cross(relPosInFrame));
      forceAngSpeedInFrame = -rho * GetVolume() * ca.cwiseProduct(forceAngSpeedInFrame);
      m_force += m_node->GetFrameInWorld().ProjectVectorFrameInParent(forceAngSpeedInFrame, NWU);
      localForce += forceAngSpeedInFrame;

      // Update added mass matrix in world
      SetAMInWorld();
    }

    // Compute the corresponding torque at COG in body reference frame
    auto forceBody = m_node->GetFrameInBody().ProjectVectorFrameInParent(localForce, NWU);
    Position relPos = m_node->GetNodePositionInBody(NWU) - body->GetCOG(NWU);
    m_torque = relPos.cross(forceBody);
  }

  void FrMorisonSingleElement::Initialize() {
    assert(m_node);
    assert(m_node->GetBody());
    assert(m_property.length > FLT_EPSILON);
    assert(m_property.diameter > FLT_EPSILON);

    SetVolume();  // FIXME : interpolation lineaire a mettre en place ?

    if (m_extendedModel) {
      SetAMInFrame();
      SetAMInBody();
    }

    CheckImmersion();
  }

  void FrMorisonSingleElement::StepFinalize() {
    // Nothing to do
  }

  void FrMorisonSingleElement::ComputeForceAddedMass() {
    // FIXME : a voir si on utilise la matrice de masse d'eau ajoutée ou ce calcul direct

    auto body = m_node->GetBody();
    auto rho = body->GetSystem()->GetEnvironment()->GetOcean()->GetDensity();

    // Initialize force and torque with null value
    m_force_added_mass.SetNull();
    m_torque_added_mass.SetNull();

    // Compute force with local node acceleration (in world reference frame)
    Acceleration node_acc = GetNodeAcceleration();
    m_force_added_mass.x() = -rho * m_property.ca.x * GetVolume() * node_acc.x();
    m_force_added_mass.y() = -rho * m_property.ca.y * GetVolume() * node_acc.y();
    m_force_added_mass.z() = 0.0;
    m_force_added_mass = m_node->GetFrameInWorld().ProjectVectorFrameInParent(m_force_added_mass, NWU);

    // Compute the corresponding torque (in body reference frame)
    Position relPos = m_node->GetPositionInWorld(NWU) - body->GetCOGPositionInWorld(NWU);
    m_torque_added_mass = relPos.cross(m_force_added_mass);

  }

  // -------------------------------------------------------------------
  // MORISON COMPOSITE FORCE MODEL
  // -------------------------------------------------------------------

  FrMorisonCompositeElement::FrMorisonCompositeElement(const std::string& name, FrBody *body, bool extendedModel)
    : FrTreeNode(name, body), m_property() {
    m_node = std::make_shared<FrNode>("", body);
    SetExtendedModel(extendedModel);
    if (m_extendedModel) {
      m_chronoPhysicsItem = std::make_shared<internal::FrMorisonModelBase>(this);
    }
  }

  FrMorisonCompositeElement::FrMorisonCompositeElement(const std::string& name, FrBody *body, FrFrame &frame, bool extendedModel)
    : FrTreeNode(name, body), m_property() {
    m_node = std::make_shared<FrNode>("", body); // TODO : Devrait etre instancie dans la classe de base
    m_node->SetFrameInBody(frame);
    SetExtendedModel(extendedModel);
    if (m_extendedModel) {
      m_chronoPhysicsItem = std::make_shared<internal::FrMorisonModelBase>(this);
    }
  }

  void
  FrMorisonCompositeElement::AddElement(std::shared_ptr<FrNode> &nodeA, std::shared_ptr<FrNode> &nodeB, double diameter,
                                        MorisonCoeff ca, MorisonCoeff cd, double cf, Direction perpendicular) {
    m_morison.push_back(std::make_unique<FrMorisonSingleElement>(nodeA, nodeB, diameter, ca, cd, cf, perpendicular));
  }

  void FrMorisonCompositeElement::AddElement(std::shared_ptr<FrNode> &nodeA, std::shared_ptr<FrNode> &nodeB,
                                             Direction perpendicular) {
    m_morison.push_back(std::make_unique<FrMorisonSingleElement>(nodeA, nodeB, m_property.diameter,
                                                                 m_property.ca, m_property.cd,
                                                                 m_property.cf, perpendicular));
  }

  void FrMorisonCompositeElement::AddElement(Position posA, Position posB, double diameter,
                                             MorisonCoeff ca, MorisonCoeff cd, double cf, unsigned int n,
                                             Direction perpendicular) {
    Direction dV = (posB - posA) / n;

    Position pos;
    for (unsigned int i = 0; i < n; ++i) {
      pos = posA + dV * i;
      m_morison.push_back(std::make_unique<FrMorisonSingleElement>(m_node->GetBody(), pos, pos + dV, diameter,
                                                                   ca, cd, cf, perpendicular));
    }
  }

  void FrMorisonCompositeElement::AddElement(Position posA, Position posB, unsigned int n, Direction perpendicular) {
    AddElement(posA, posB, m_property.diameter, m_property.ca, m_property.cd, m_property.cf, n, perpendicular);
  }

  void FrMorisonCompositeElement::AddElement(FrFrame frame, double length, double diameter,
                                             MorisonCoeff ca, MorisonCoeff cd, double cf) {
    m_morison.push_back(std::make_unique<FrMorisonSingleElement>(m_node->GetBody(), frame, diameter,
                                                                 length, ca, cd, cf));
  }

  void FrMorisonCompositeElement::AddElement(FrFrame frame, double length) {
    AddElement(frame, length, m_property.diameter, m_property.ca, m_property.cd, m_property.cf);
  }

  void FrMorisonCompositeElement::SetDragCoeff(MorisonCoeff cd) {
    m_property.cd = cd;
  }

  void FrMorisonCompositeElement::SetFrictionCoeff(double cf) {
    m_property.cf = cf;
  }

  void FrMorisonCompositeElement::SetAddedMass(MorisonCoeff ca) {
    m_property.ca = ca;
  }

  void FrMorisonCompositeElement::SetDiameter(double diameter) {
    m_property.diameter = diameter;
  }

  void FrMorisonCompositeElement::Initialize() {

    for (auto &element: m_morison) {
      element->SetExtendedModel(m_extendedModel);
      element->Initialize();
    }

    m_AMInBody.setZero();
    m_AMInWorld.setZero();
    // TODO : not mandatory loop
    for (auto& element: m_morison) {
      if (element->IsImmerged()) {
        m_AMInBody += element->GetAMInBody();
        m_AMInWorld += element->GetAMInWorld();
      }
    }

    if (m_extendedModel) {
      m_chronoPhysicsItem->SetupInitial();
    }

  }

  void FrMorisonCompositeElement::Compute(double time) {
    this->Update(time);
  }

  void FrMorisonCompositeElement::Update(double time) {

    m_force.SetNull();
    m_torque.SetNull();
    m_isImmerged = false;

    for (auto &element : m_morison) {
      element->Update(time);
      m_force += element->GetForceInWorld(NWU);
      m_torque += element->GetTorqueInBody();
      m_isImmerged = m_isImmerged or element->IsImmerged();
    }

    if (m_extendedModel) {
      m_AMInBody.setZero();
      m_AMInWorld.setZero();
      for (auto& element: m_morison) {
        if (element->IsImmerged()) {
          m_AMInBody += element->GetAMInBody();
          m_AMInWorld += element->GetAMInWorld();
        }
      }
      m_chronoPhysicsItem->Update(time, false);
    }
  }

  void FrMorisonCompositeElement::ComputeForceAddedMass() {
    // FIXME : a voir si on utilise la matrice de masse d'eau ajoutée ou le calcul direct
    // Added mass force and torque
    m_force_added_mass.SetNull();
    m_torque_added_mass.SetNull();

    for (auto& element: m_morison) {
      if (element->IsImmerged()) {
        element->ComputeForceAddedMass();
        m_force_added_mass += element->GetForceAddedMassInWorld(NWU);
        m_torque_added_mass += element->GetTorqueAddedMassInWorld();
      }
    }

  }

}  // end namespace frydom
