//
// Created by frongere on 30/04/2020.
//

#include "FrFEALink.h"


namespace frydom {

  namespace internal {


    FrFEALinkBase::FrFEALinkBase(bool mc_x,
                                 bool mc_y,
                                 bool mc_z,
                                 bool mc_rx,
                                 bool mc_ry,
                                 bool mc_rz)
        : chrono::ChLinkMateGeneric(mc_x,
                            mc_y,
                            mc_z,
                            mc_rx,
                            mc_ry,
                            mc_rz) {}



  }  // end namespace frydom::internal


}  // end namespace frydom
