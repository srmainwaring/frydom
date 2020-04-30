//
// Created by frongere on 29/04/2020.
//

#include "FrFEANode.h"


namespace frydom {

  namespace internal {

    FrFEANodeBase::FrFEANodeBase(chrono::ChFrame<> frame) :
        chrono::fea::ChNodeFEAxyzrot(frame) {}


  }  // end namespace frydom::internal

}  // end namespace frydom
