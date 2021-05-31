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

#ifndef FRYDOM_FRCASTOR_H
#define FRYDOM_FRCASTOR_H

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace frydom {

  class FrCastorParameters {

   public:

    FrCastorParameters();

    void Write(const std::string& path);

    void Add(const json& node);

   private:

    std::string m_filename = "castor.json";

    json m_json_object;

  };

} // end namespace frydom

#endif //FRYDOM_FRCASTOR_H
