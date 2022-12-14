//
// Created by frongere on 02/08/2021.
//

#ifndef FRYDOM_FRPROPELLERTYPE_H
#define FRYDOM_FRPROPELLERTYPE_H

#include "acme/propeller/PropellerModelType.h"
#include "acme/propeller/PropellerBaseModel.h"
#include "acme/rudder/RudderModelType.h"
#include "acme/rudder/SimpleRudderModel.h"
#include "acme/propeller_rudder/PropellerRudderBase.h"

namespace frydom {

  using PropellerModelType = acme::PropellerModelType;
  using PropellerParams = acme::PropellerParams;
  using PropellerRudderModelType = acme::PropellerRudderModelType;
  using SCREW_DIRECTION = acme::SCREW_DIRECTION;

//  using PropellerModelType = acme::PropellerModelType;

  using RudderModelType = acme::RudderModelType;
  using RudderParams = acme::RudderParams;

//  using RudderModelType = acme::RudderModelType;

}  // end namespace frydom


#endif //FRYDOM_FRPROPELLERTYPE_H
