//
// Created by frongere on 03/08/2021.
//

#include "FrPropulsionActuator.h"

namespace frydom {

  frydom::FrPropulsionActuator::FrPropulsionActuator(const std::string &name,
                                                     const std::string &type_name,
                                                     FrBody *body) :
      FrForce(name, type_name, body) {

  }

}  // end namespace frydom