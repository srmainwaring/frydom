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


#ifndef FRYDOM_FRLOGMANAGER_H
#define FRYDOM_FRLOGMANAGER_H

#include <memory>

#include <string>
#include <list>

#include "FrCastorManager.h"


namespace frydom {

  // Forward declaration
  class FrLoggableBase;

  class FrOffshoreSystem;

  class FrLogManager {

   public:

    explicit FrLogManager(FrOffshoreSystem *system, const std::string &logFolderName);

    ~FrLogManager();

    FrOffshoreSystem *GetSystem() const;

    void Add(const std::shared_ptr<FrLoggableBase> &obj);

    void Add(FrLoggableBase *obj);

    void Remove(const std::shared_ptr<FrLoggableBase> &obj);

    void Remove(FrLoggableBase *obj);

    unsigned int GetNumberOfLoggables() const;

    void SetNFreqOutput(int n);

    virtual void Initialize();

    void StepFinalize();

    void SetLogFrameConvention(FRAME_CONVENTION fc);

    void NoCSVLlog();  // TODO: permettre de ne pas logger en CSV... -> perf !

    void LogCSV(bool val);

    void LogHDF5(bool val);

    void DisableAllLogs();

    using LoggableList = std::list<FrLoggableBase *>;
    using LoggableIter = LoggableList::iterator;

    LoggableIter begin() {
      return m_loggable_list.begin();
    };

    LoggableIter end() {
      return m_loggable_list.end();
    }

    static std::string GetDateFolder();

    FrCastorManager& GetCastorParameters();

   private:
    bool Has(FrLoggableBase *obj) const;

//    std::string InitializeLogFolder();

//    static std::string LogFolderFromFrydomConfigFile(const std::string &path_to_config_file);

    void WriteMetaDataFile() const;

    void WriteCastorFile();

    static std::string now();

   private:

    std::string m_log_folder;

    LoggableList m_loggable_list;

    FrOffshoreSystem *m_system;

    FrCastorManager m_castor;

    bool m_log_CSV;
    bool m_log_HDF5;

    int m_nfreq_output;
    int m_ifreq_output;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };


}  // end namespace frydom



#endif //FRYDOM_FRLOGMANAGER_H
