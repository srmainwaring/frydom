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


namespace frydom {

  // Forward declaration
  class FrLoggableBase;

  class FrOffshoreSystem;


  class FrLogManager {

   public:

    explicit FrLogManager(FrOffshoreSystem *system);

    FrLogManager(const std::string &log_folder, FrOffshoreSystem *system);

    FrOffshoreSystem *GetSystem() const;

    const std::string &GetLogFolder() const;

    void Add(const std::shared_ptr<FrLoggableBase> &obj);

    void Add(FrLoggableBase *obj);

    void Remove(const std::shared_ptr<FrLoggableBase> &obj);

    void Remove(FrLoggableBase *obj);

    unsigned int GetNumberOfLoggables() const;

    virtual void Initialize();

    void StepFinalize();

    void SetLogFrameConvention(FRAME_CONVENTION fc);

    void LogCSV(bool val);  // TODO: permettre de ne pas logger en CSV... -> perf !


   private:
    bool Has(FrLoggableBase *obj) const;

    std::string InitializeLogFolder();


   private:
    std::string m_log_folder;

    std::list<FrLoggableBase *> m_loggable_list;

    bool m_log_CSV;

//    std::vector<std::unique_ptr<hermes::Serializer>> m_serializers;

  };

}  // end namespace frydom



#endif //FRYDOM_FRLOGMANAGER_H
