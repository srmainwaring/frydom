//
// Created by frongere on 02/08/2021.
//

#ifndef FRYDOM_FRPROPELLERTYPE_H
#define FRYDOM_FRPROPELLERTYPE_H

namespace frydom {

  enum PROPELLER_TYPE {
    FPP_1Q,  // Fixed Propeller Pitch with one quadrant model
    FPP_4Q,  // Fixed Propeller Pitch with four quadrant model
    CPP      // Controllable Pitch Propeller (always four quadrant model extended with pitch ratio dependency)
  };

  enum SCREW_DIRECTION {
    LEFT_HANDED,
    RIGHT_HANDED
  };

}  // end namespace frydom


#endif //FRYDOM_FRPROPELLERTYPE_H
