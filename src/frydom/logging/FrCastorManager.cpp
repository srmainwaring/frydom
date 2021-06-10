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

#include <iostream>
#include <fstream>
#include "frydom/utils/FrFileSystem.h"
#include "FrCastorManager.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/environment/FrEnvironmentInc.h"


namespace frydom {

  FrCastorManager::FrCastorManager(FrOffshoreSystem* system) : m_system(system) {}

  void FrCastorManager::Write(const std::string& path) {

    auto filename = FrFileSystem::join({path, m_filename});

    m_json_object["DataFolder"] = m_system->config_file().GetDataFolder();

    for (auto& body: m_system->GetBodyList()) {
      json j;
      j["mesh_filename"] = body->GetMeshFilename();

      auto pos = body->GetMeshOffsetPosition(NWU);
      double phi, theta, psi;
      body->GetMeshOffsetRotation().GetCardanAngles_RADIANS(phi, theta, psi, NWU);
      j["mesh_offset"] = {pos.x(), pos.y(), pos.y(), phi, theta, psi};
      m_json_object[body->GetName()] = j;
    }

    auto free_surface = m_system->GetEnvironment()->GetOcean()->GetFreeSurface();
    auto wave = free_surface->GetWaveField();
    SetParametersJSON(wave);

    std::ofstream file;
    file.open(filename, std::ios::trunc);
    file << m_json_object.dump(2);
    file.close();

  }

  void FrCastorManager::SetParametersJSON(FrWaveField* wave) {

    json j;

    if (auto irregular_wave = dynamic_cast<FrAiryIrregularWaveField*>(wave)) {
      j["wave_field_type"] = "AiryIrregularWaveField";
      j["wave_frequencies_rads"] = irregular_wave->GetWaveFrequencies(RADS);
      j["wave_numbers_m"] = irregular_wave->GetWaveNumbers();
      j["wave_directions_rad"] = irregular_wave->GetWaveDirections(RAD, NWU, GOTO);
      j["wave_mean_direction_rad"] = irregular_wave->GetMeanWaveDirectionAngle(RAD, NWU, GOTO);
      j["wave_amplitudes_m"] = irregular_wave->GetWaveAmplitudes();
      j["wave_phases_rad"] = irregular_wave->GetWavePhases();
      m_json_object["AiryIrregularWaveField"] = j;

    } else if (auto regular_wave = dynamic_cast<FrAiryRegularWaveField*>(wave)) {
      j["wave_field_type"] = "AiryRegularWaveField";
      j["wave_amplitude_m"] = regular_wave->GetWaveAmplitudes();
      j["wave_frequency_rads"] = regular_wave->GetWaveFrequencies(RADS);
      j["wave_direction_rad"] = regular_wave->GetDirectionAngle(RAD, NWU, GOTO);
      j["wave_number_m"] = regular_wave->GetWaveNumbers();
      m_json_object["AiryRegularWaveField"] = j;

    }
  }

} // end namespace frydom

