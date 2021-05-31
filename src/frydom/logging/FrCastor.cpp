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

#include <fstream>
#include "frydom/utils/FrFileSystem.h"
#include "FrCastor.h"

namespace frydom {

  FrCastorParameters::FrCastorParameters() {
  }

  void FrCastorParameters::Write(const std::string& path) {

    auto filename = FrFileSystem::join({path, m_filename});

    std::ofstream file;
    file.open(filename, std::ios::trunc);
    file << m_json_object.dump(2);
    file.close();

  }

  void FrCastorParameters::Add(const json& node) {
    m_json_object += node;
  }


} // end namespace frydom

