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


#include "FrCable.h"
#include "frydom/core/common/FrObject.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/logging/FrLogManager.h"


namespace frydom {


  FrCableProperties::FrCableProperties(double diameter, double linearDensity, double youngModulus) :
      m_youngModulus(youngModulus), m_linearDensity(linearDensity) {
    SetDiameter(diameter);
  }

  FrCableProperties::FrCableProperties(double diameter, double linearDensity, double youngModulus,
                                       double rayleighDamping) :
      FrCableProperties(diameter, linearDensity, youngModulus) {
    m_rayleighDamping = rayleighDamping;
  }

  void FrCableProperties::SetYoungModulus(double E) {
    m_youngModulus = E;
  }

  double FrCableProperties::GetYoungModulus() const {
    return m_youngModulus;
  }

  void FrCableProperties::SetSectionArea(double A) {
    m_section = A;
  }

  double FrCableProperties::GetSectionArea() const {
    return m_section;
  }

  void FrCableProperties::SetDiameter(double d) {
    m_section = 0.25 * M_PI * d * d;
  }

  double FrCableProperties::GetDiameter() const {
    return sqrt(4. * m_section / M_PI);
  }

  double FrCableProperties::GetRadius() const {
    return 0.5 * GetDiameter();
  }

  void FrCableProperties::SetEA(double EA) {
    m_youngModulus = EA / m_section;
  }

  double FrCableProperties::GetEA() const {
    return m_youngModulus * m_section;
  }

  void FrCableProperties::SetLinearDensity(double lambda) {
    m_linearDensity = lambda;
  }

  double FrCableProperties::GetLinearDensity() const {
    return m_linearDensity;
  }

  void FrCableProperties::SetDensity(double rho) {
    m_linearDensity = rho * m_section;
  }

  double FrCableProperties::GetDensity() const {
    return m_linearDensity / m_section;
  }

  void FrCableProperties::SetBreakingTension(double breakingTension) {
    assert(breakingTension > DBL_EPSILON);
    m_breakingTension = breakingTension;
  }

  double FrCableProperties::GetBreakingTension() const {
    return m_breakingTension;
  }

  double FrCableProperties::GetRayleighDamping() const {
    return m_rayleighDamping;
  }

  std::shared_ptr<FrCableProperties> make_cable_properties() {
    return std::make_shared<FrCableProperties>();
  }

  std::shared_ptr<FrCableProperties> make_cable_properties(double diameter, double linearDensity, double youngModulus) {
    return std::make_shared<FrCableProperties>(diameter, linearDensity, youngModulus);
  }

  std::shared_ptr<FrCableProperties> make_cable_properties(double diameter,
                                                           double linearDensity,
                                                           double youngModulus,
                                                           double rayleighDamping) {
    return std::make_shared<FrCableProperties>(diameter, linearDensity, youngModulus, rayleighDamping);
  }

  //------------------------------------------------------------------------------------------------------------------
  // FrCable

  FrCable::FrCable(const std::shared_ptr<FrNode> &startingNode,
                   const std::shared_ptr<FrNode> &endingNode) :
      m_startingNode(startingNode),
      m_endingNode(endingNode) {

    m_properties = std::make_shared<FrCableProperties>();
  }

  FrCable::FrCable(const std::shared_ptr<FrNode> &startingNode,
                   const std::shared_ptr<FrNode> &endingNode,
                   const std::shared_ptr<FrCableProperties> &properties,
                   double unstretchedLength) :
      m_startingNode(startingNode),
      m_endingNode(endingNode),
      m_unstretchedLength(unstretchedLength),
      m_properties(properties) {}

  void FrCable::Initialize() {}

  void FrCable::SetCableProperties(const std::shared_ptr<FrCableProperties> prop) {
    m_properties = prop;
  }

  std::shared_ptr<FrCableProperties> FrCable::GetCableProperties() const {
    return m_properties;
  }

  void FrCable::SetUnstretchedLength(double L) {
    m_unstretchedLength = L;
    BuildCache();
  }

  double FrCable::GetUnstretchedLength() const {
    return m_unstretchedLength;
  }


  void FrCable::SetStartingNode(const std::shared_ptr<FrNode> startingNode) {
    // TODO: permettre de re-attacher le cable a un autre noeud si elle etait deja attachee a un noeud
    m_startingNode = startingNode;
  }

  std::shared_ptr<FrNode> FrCable::GetStartingNode() const {
    return m_startingNode;
  }

  void FrCable::SetEndingNode(const std::shared_ptr<FrNode> endingNode) {
    // TODO: permettre de re-attacher le cable a un autre noeud si elle etait deja attachee a un noeud
    m_endingNode = endingNode;
  }

  std::shared_ptr<FrNode> FrCable::GetEndingNode() const {
    return m_endingNode;
  }

//    void FrCable::SetBreakingTension(double tension) {
//        m_breakingTension = tension;
//    }
//
//    double FrCable::GetBreakingTension() const {
//        return m_breakingTension;
//    }

//    void FrCable::InitBreakingTension() {
//
//        if (GetBreakingTension()==0){
//            double ds = GetUnstretchedLength()/ GetAssetElements();
//            double max = GetTension(0, NWU).norm();
//            for (int i=1; i< GetAssetElements(); i++){
//                auto LocalTension = GetTension(i*ds, NWU).norm();
//                if (LocalTension > max) max = LocalTension;
//            }
//            SetBreakingTension(1.25*max);  // TODO : affiner le critere...
//        }
//
//    }

  void FrCable::SetUnrollingSpeed(double unrollingSpeed) {
    m_unrollingSpeed = unrollingSpeed;
  }

  double FrCable::GetUnrollingSpeed() const {
    return m_unrollingSpeed;
  }

  void FrCable::UpdateTime(double time) {
    m_time_step = time - m_time;
    m_time = time;
  }

  void FrCable::UpdateState() {
    if (std::abs(m_unrollingSpeed) > DBL_EPSILON and std::abs(m_time_step) > DBL_EPSILON) {
      m_unstretchedLength += m_unrollingSpeed * m_time_step;
    }
  }

  double FrCable::GetStrainedLength() const { // FIXME: ne fonctionne pas, retourne 0. !!
    double cl = 0.;
    int n = 1000;

    double ds = GetUnstretchedLength() / (n - 1);
    auto pos_prev = GetPositionInWorld(0., NWU);

    for (uint i = 0; i < n; ++i) {
      auto s = i * ds;
      auto pos = GetPositionInWorld(s, NWU);
      cl += (pos - pos_prev).norm();
      pos_prev = pos;
    }
    return cl;
  }

//  FrOffshoreSystem *FrCable::GetSystem() const {
//    return GetParent();
//  }

}  // end namespace frydom
