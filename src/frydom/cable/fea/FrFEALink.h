//
// Created by frongere on 30/04/2020.
//

#ifndef FRYDOM_FRFEALINK_H
#define FRYDOM_FRFEALINK_H

#include <chrono/physics/ChLinkMate.h>

namespace frydom {

  namespace internal {

    class FrFEALinkBase : public chrono::ChLinkMateGeneric {
     public:
      FrFEALinkBase(bool mc_x = true,
                    bool mc_y = true,
                    bool mc_z = true,
                    bool mc_rx = true,
                    bool mc_ry = true,
                    bool mc_rz = true);

    };

  }  // end namspace frydom::internal


//  class FrFEALink {
//
//   public:
//    FrFEALink();
//
//   private:
//    std::shared_ptr<internal::FrFEALinkBase> m_chrono_link;
//
//
//  };

}  // end namespace frydom



#endif //FRYDOM_FRFEALINK_H
