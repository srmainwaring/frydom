//
// Created by frongere on 29/04/2020.
//

#ifndef FRYDOM_FRFEANODE_H
#define FRYDOM_FRFEANODE_H

#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/core/ChFrame.h"



namespace frydom {

  namespace internal {

    class FrFEANodeBase : public chrono::fea::ChNodeFEAxyzrot {

     public:

      FrFEANodeBase(chrono::ChFrame<> frame);



     private:


    };

  }  // end namespace frydom::internal

}  // end namespace frydom



#endif //FRYDOM_FRFEANODE_H
