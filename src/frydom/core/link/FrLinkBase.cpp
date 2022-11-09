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

#include "FrLinkBase.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/core/common/FrNode.h"


namespace frydom {
  FrLinkBase::~FrLinkBase() {
  }

  FrLinkBase::FrLinkBase(const std::shared_ptr<FrNode> &node1,
                         const std::shared_ptr<FrNode> &node2) :
      m_node1(node1),
      m_node2(node2) {}

  std::shared_ptr<FrNode> FrLinkBase::GetNode1() {
    return m_node1;
  }

  const std::shared_ptr<FrNode> FrLinkBase::GetNode1() const {
    return m_node1;
  }

  std::shared_ptr<FrNode> FrLinkBase::GetNode2() {
    return m_node2;
  }

  const std::shared_ptr<FrNode> FrLinkBase::GetNode2() const {
    return m_node2;
  }

  FrBody *FrLinkBase::GetBody1() {
    return m_node1->GetBody();
  }

  FrBody *FrLinkBase::GetBody2() {
    return m_node2->GetBody();
  }


  namespace internal {

    std::shared_ptr<FrBodyBase> GetChronoBody1(std::shared_ptr<FrLinkBase> link) {
      return internal::GetChronoBody(link->GetNode1()->GetBody());
    }

    std::shared_ptr<FrBodyBase> GetChronoBody1(FrLinkBase *link) {
      return internal::GetChronoBody(link->GetNode1()->GetBody());
    }

    std::shared_ptr<FrBodyBase> GetChronoBody2(std::shared_ptr<FrLinkBase> link) {
      return internal::GetChronoBody(link->GetNode2()->GetBody());
    }

    std::shared_ptr<FrBodyBase> GetChronoBody2(FrLinkBase *link) {
      return internal::GetChronoBody(link->GetNode2()->GetBody());
    }

  }  // end namespace frydom::internal


}  // end namespace frydom
