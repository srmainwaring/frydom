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


#include "FrCableBase.h"
#include "FrCableProperties.h"
#include "frydom/core/common/FrObject.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/logging/FrLogManager.h"


namespace frydom {

  //------------------------------------------------------------------------------------------------------------------
  // FrCableBase

  FrCableBase::FrCableBase(const std::shared_ptr<FrNode> &startingNode,
                           const std::shared_ptr<FrNode> &endingNode) :
      m_startingNode(startingNode),
      m_endingNode(endingNode) {

    m_properties = std::make_shared<FrCableProperties>();
  }

  FrCableBase::FrCableBase(const std::shared_ptr<FrNode> &startingNode,
                           const std::shared_ptr<FrNode> &endingNode,
                           const std::shared_ptr<FrCableProperties> &properties,
                           double unstretchedLength) :
      m_startingNode(startingNode),
      m_endingNode(endingNode),
      m_unstretchedLength(unstretchedLength),
      m_properties(properties) {}

  void FrCableBase::Initialize() {}

  void FrCableBase::SetCableProperties(const std::shared_ptr<FrCableProperties> prop) {
    m_properties = prop;
  }

  std::shared_ptr<FrCableProperties> FrCableBase::GetProperties() const {
    return m_properties;
  }

  void FrCableBase::SetUnstretchedLength(double L) {
    m_unstretchedLength = L;
    BuildCache();
  }

  double FrCableBase::GetUnstretchedLength() const {
    return m_unstretchedLength;
  }


  void FrCableBase::SetStartingNode(const std::shared_ptr<FrNode> startingNode) {
    // TODO: permettre de re-attacher le cable a un autre noeud si elle etait deja attachee a un noeud
    m_startingNode = startingNode;
  }

  std::shared_ptr<FrNode> FrCableBase::GetStartingNode() const {
    return m_startingNode;
  }

  void FrCableBase::SetEndingNode(const std::shared_ptr<FrNode> endingNode) {
    // TODO: permettre de re-attacher le cable a un autre noeud si elle etait deja attachee a un noeud
    m_endingNode = endingNode;
  }

  std::shared_ptr<FrNode> FrCableBase::GetEndingNode() const {
    return m_endingNode;
  }

//    void FrCableBase::SetBreakingTension(double tension) {
//        m_breakingTension = tension;
//    }
//
//    double FrCableBase::GetBreakingTension() const {
//        return m_breakingTension;
//    }

//    void FrCableBase::InitBreakingTension() {
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

  void FrCableBase::SetUnrollingSpeed(double unrollingSpeed) {
    m_unrollingSpeed = unrollingSpeed;
  }

  double FrCableBase::GetUnrollingSpeed() const {
    return m_unrollingSpeed;
  }

  void FrCableBase::UpdateTime(double time) {
    m_time_step = time - m_time;
    m_time = time;
  }

  void FrCableBase::UpdateState() {
    if (std::abs(m_unrollingSpeed) > DBL_EPSILON and std::abs(m_time_step) > DBL_EPSILON) {
      m_unstretchedLength += m_unrollingSpeed * m_time_step;
    }
  }

  double FrCableBase::GetStrainedLength() const { // FIXME: ne fonctionne pas, retourne 0. !!
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

//  FrOffshoreSystem *FrCableBase::GetSystem() const {
//    return GetParent();
//  }

}  // end namespace frydom
