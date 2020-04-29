//
// Created by frongere on 29/04/2020.
//

#include "FrFEANode.h"


namespace frydom {

  namespace internal {

    FrFEANodeBase::FrFEANodeBase(const FrFrame &frame) :
        chrono::fea::ChNodeFEAxyzrot(internal::FrFrame2ChFrame(frame)) {}


  }  // end namespace frydom::internal



  FrFEANode::FrFEANode() :
      m_chrono_node(std::make_shared<internal::FrFEANodeBase>(FrFrame())) {}

  FrFEANode::FrFEANode(const FrFrame &frame) :
      m_chrono_node(std::make_shared<internal::FrFEANodeBase>(frame)) {}


}  // end namespace frydom
