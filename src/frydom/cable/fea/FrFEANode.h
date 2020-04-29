//
// Created by frongere on 29/04/2020.
//

#ifndef FRYDOM_FRFEANODE_H
#define FRYDOM_FRFEANODE_H

#include "chrono/fea/ChNodeFEAxyzrot.h"


#include "frydom/core/common/FrFrame.h"


namespace frydom {

  namespace internal {

    class FrFEANodeBase : public chrono::fea::ChNodeFEAxyzrot {

     public:

      FrFEANodeBase(const FrFrame &frame);

     private:


    };

  }  // end namespace frydom::internal


  class FrFEANode {

   public:
    FrFEANode();

    explicit FrFEANode(const FrFrame &frame);


   private:
    std::shared_ptr<internal::FrFEANodeBase> m_chrono_node;

  };

}  // end namespace frydom



#endif //FRYDOM_FRFEANODE_H
