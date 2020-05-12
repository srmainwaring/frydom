//
// Created by frongere on 30/04/2020.
//

#include "FrFEALink.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"
#include "FrFEANode.h"


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


    void FrFEANodeBodyDistance::Initialize(std::shared_ptr<FrFEANodeBase> fea_node,
                                           std::shared_ptr<FrNode> body_node) {

      assert(false);
//      Initialize(fea_node,
//          body_node->GetBody()->GetChronoBody())
// TODO: continuer

    }


  }  // end namespace frydom::internal


}  // end namespace frydom
