//
// Created by frongere on 03/08/2021.
//

#ifndef FRYDOM_FRACTUATORFORCEBASE_H
#define FRYDOM_FRACTUATORFORCEBASE_H

#include "frydom/core/force/FrForce.h"

namespace frydom {

  class FrActuatorForceBase : public FrForce {

   public:
    FrActuatorForceBase(const std::string &name,
                        const std::string &type_name,
                        FrBody *body);

  };

}  // end namespace frydom

#endif //FRYDOM_FRACTUATORFORCEBASE_H
