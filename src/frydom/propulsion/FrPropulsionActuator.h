//
// Created by frongere on 03/08/2021.
//

#ifndef FRYDOM_FRPROPULSIONACTUATOR_H
#define FRYDOM_FRPROPULSIONACTUATOR_H

#include "frydom/core/force/FrForce.h"

namespace frydom {

  class FrPropulsionActuator : public FrForce {

   public:
    FrPropulsionActuator(const std::string &name,
                         const std::string &type_name,
                         FrBody *body);

  };

}  // end namespace frydom

#endif //FRYDOM_FRPROPULSIONACTUATOR_H
