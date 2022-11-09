//
// Created by frongere on 03/08/2021.
//

#include "FrActuatorForceBase.h"

namespace frydom {

  FrActuatorForceBase::~FrActuatorForceBase() {
  }

  FrActuatorForceBase::FrActuatorForceBase(const std::string &name,
                                           const std::string &type_name,
                                           FrBody *body) :
      FrForce(name, type_name, body) {

  }

}  // end namespace frydom