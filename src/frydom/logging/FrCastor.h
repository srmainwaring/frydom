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

    FrCastorParameters(const std::string& folder);

    void Write(const std::string& path);

    void Add(const std::string& tag, const json& node);

    void SetDataFolder(const std::string& folder);

   private:

    std::string m_filename = "castor.json";

    json m_json_object;

  };

} // end namespace frydom

#endif //FRYDOM_FRCASTOR_H
