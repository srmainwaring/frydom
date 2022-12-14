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

#include <algorithm>
#include <nlohmann/json.hpp>
#include <ctime>
#include <chrono>

#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/common/FrConvention.h"
#include "frydom/core/FrPlatform.h"
#include "frydom/utils/FrFileSystem.h"
#include "frydom/version.h"

#include "FrPathManager.h"
#include "FrLogManager.h"
#include "FrLoggable.h"
#include "FrEventLogger.h"

#include "FrCastorManager.h"


#define META_FILE_NAME "meta.json"
#define DATE_FOLDER_FORMAT "%Y-%m-%d_%Hh%Mm%Ss"


using json = nlohmann::json;

namespace frydom {

  FrLogManager::FrLogManager(FrOffshoreSystem *system, const std::string &logFolderName) :
      m_log_CSV(true),
      m_system(system),
      m_nfreq_output(1),
      m_ifreq_output(0),
      m_castor(system) { // FIXME : Du coup LogManager devrait etre un TreeNode...

    Add(system);
    m_log_folder = FrFileSystem::join({system->config_file().GetLogFolder(), logFolderName});
    FrFileSystem::mkdir(m_log_folder);

    WriteMetaDataFile();

    event_logger::info("LogManager", "", "Logging into directory \"{}\".", m_log_folder);

    // Event Logger initialization
    event_logger::init(system, GetSystem()->GetName(), FrFileSystem::join({m_log_folder, "events.txt"}));

  }

  FrLogManager::~FrLogManager() {
    event_logger::reset_to_default_logger();
  };

  FrOffshoreSystem *FrLogManager::GetSystem() const {
    return m_system;
  }

  void FrLogManager::Add(const std::shared_ptr<FrLoggableBase> &obj) {
    Add(obj.get());
  }

  void FrLogManager::Add(FrLoggableBase *obj) {
    if (!Has(obj)) m_loggable_list.push_back(obj);
  }

  void FrLogManager::Remove(const std::shared_ptr<FrLoggableBase> &obj) {
    Remove(obj.get());
  }

  void FrLogManager::Remove(FrLoggableBase *obj) {
    auto it = std::find(m_loggable_list.begin(), m_loggable_list.end(), obj);

    // Remove if present
    if (it != m_loggable_list.end()) {
      m_loggable_list.erase(it);
    }
  }

  bool FrLogManager::Has(FrLoggableBase *obj) const {
    return (std::find(m_loggable_list.begin(), m_loggable_list.end(), obj) != m_loggable_list.end());
  }

  void FrLogManager::WriteMetaDataFile() const {

    json j;

    j["date"] = now();
    j["username@hostname"] = FrFileSystem::get_login() + "@" + FrFileSystem::get_hostname();
    j["project_name"] = GetSystem()->GetName();
    j["frydom_git_revision"] = git::GetNormalizedVersionString();
    j["platform"] = GetPlatformName();


    std::ofstream file;
    file.open(FrFileSystem::join({m_log_folder, META_FILE_NAME}), std::ios::trunc);
    file << j.dump(2);
    file.close();

  }

  void FrLogManager::WriteCastorFile() {
    m_castor.Write(m_log_folder);
  }

  std::string FrLogManager::now() { // TODO : voir a faire avec fmt...
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string now_str = std::ctime(&now);
    return now_str.substr(0, now_str.size() - 1); // Removing last char that is an end line...
  }

//  std::string FrLogManager::LogFolderFromFrydomConfigFile(const std::string &path_to_config_file) {
//
//    if (!FrFileSystem::exists(path_to_config_file)) exit(EXIT_FAILURE);
//
//    std::ifstream ifs(path_to_config_file);
//
//    json json_obj = json::parse(ifs);
//
//    std::string log_folder;
//
//    try {
//      log_folder = json_obj["log_folder"].get<std::string>();
//    } catch (nlohmann::detail::type_error &e) {
//      log_folder = FrFileSystem::cwd();
//      event_logger::warn("LogManager", "",
//                         "No log_folder key found into config file \"{}\". Using current directory by default \"{}\"",
//                         path_to_config_file, log_folder);
//    }
//
//    // FIXME : on autorise pas les chemins relatif mais on devrait. Demande reflexion...
//    if (!FrFileSystem::isabs(log_folder)) return "";
//
//    return log_folder;
//
//  }

  std::string FrLogManager::GetDateFolder() {

    // TODO : utiliser fmt pour ce formatage

    time_t temps;
    struct tm datetime;
    char format[32];

    time(&temps);
    datetime = *localtime(&temps);

    strftime(format, 32, DATE_FOLDER_FORMAT, &datetime);

    return format;
  }

  FrCastorManager& FrLogManager::GetCastorParameters() {
    return m_castor;
  }

  void FrLogManager::Initialize() { // TODO : retirer la necessite d'avoir cette methode friend de FrLoggableBase

    if (m_log_CSV) {
      event_logger::info("FrLogManager", "", "CSV logging *IS* activated");
    } else {
      event_logger::info("FrLogManager", "", "CSV logging *IS NOT* activated");
    }

    for (auto &obj : m_loggable_list) {

      if (!obj->IsLogged()) {
        event_logger::info(obj->GetTypeName(), "", "won't be logged");
        continue;
      }

      obj->DefineLogMessages();

      if (m_log_CSV) {

        // Adding a CSV serializer to messages
        for (auto &message : obj->m_messages) { // TODO : ajouter un iterateur de message sur FrLoggableBase

          // Building the message folder
          std::string message_folder = FrFileSystem::join({m_log_folder, obj->GetTreePath()});
          FrFileSystem::mkdir(message_folder);

          std::string csv_file = FrFileSystem::join(
              {message_folder, obj->GetTypeName() + message->GetName() + ".csv"});

          message->AddSerializer(new hermes::CSVSerializer(csv_file));
        }

      }

      obj->InitializeLogMessages();

      obj->SendLogMessages();

    }
    WriteCastorFile();
  }

  void FrLogManager::StepFinalize() {

    m_ifreq_output += 1;

    if (m_ifreq_output == m_nfreq_output) {
      for (auto &obj : m_loggable_list) {
        if (!obj->IsLogged()) continue;
        obj->StepFinalizeLog();
      }
      m_ifreq_output = 0;
    }

  }

  void FrLogManager::SetLogFrameConvention(FRAME_CONVENTION fc) {
    for (auto &obj : m_loggable_list) {
      obj->SetLogFrameConvention(fc);
    }
  }

  void FrLogManager::NoCSVLlog() { m_log_CSV = false; }

  void FrLogManager::LogCSV(bool val) { m_log_CSV = val; }

  void FrLogManager::LogHDF5(bool val) { m_log_HDF5 = true; }

  unsigned int FrLogManager::GetNumberOfLoggables() const {
    return m_loggable_list.size();
  }

  void FrLogManager::DisableAllLogs() {
    for (auto &obj : m_loggable_list) {
      obj->LogThis(false);
    }
  }

  void FrLogManager::SetNFreqOutput(int n) {
    m_nfreq_output = n;
  }

}  // end namespace frydom
