//
// Created by frongere on 04/08/2021.
//

#ifndef ACME_FPP4Q_H
#define ACME_FPP4Q_H

#include <string>
#include "MathUtils/LookupTable1D.h"

#include "ThrusterBaseModel.h"

namespace acme {

  /// Four Quadrant model for Fixed Pitch Propeller
  class FPP4Q : public ThrusterBaseModel {

   public:
    FPP4Q(const ThrusterBaseParams &params, const std::string &kt_kq_file_json)

  };

}  // end namespace acme

#endif //ACME_FPP4Q_H
