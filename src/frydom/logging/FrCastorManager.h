// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================

#ifndef FRYDOM_FRCASTORMANAGER_H
#define FRYDOM_FRCASTORMANAGER_H

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace frydom {

  // forward declaration
  class FrOffshoreSystem;
  class FrWaveField;

  class FrCastorManager {

   public:

    explicit FrCastorManager(FrOffshoreSystem* system);

    void Write(const std::string& path);

   protected:

    void SetParametersJSON(FrWaveField* wave);

   private:

    FrOffshoreSystem* m_system;

    std::string m_filename = "castor.json";

    json m_json_object;

   //public:
   // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

} // end namespace frydom

#endif //FRYDOM_FRCASTORMANAGER_H
