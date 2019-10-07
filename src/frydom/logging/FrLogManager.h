//
// Created by frongere on 25/09/19.
//

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

    explicit FrLogManager(FrOffshoreSystem* system);

    FrLogManager(const std::string &log_folder, FrOffshoreSystem* system);

    const std::string GetLogFolder() const;

    void Add(std::shared_ptr<FrLoggableBase> obj);

    void Remove(std::shared_ptr<FrLoggableBase> obj);

    int GetNumberOfLoggables() const;

    void Initialize();

    void StepFinalize();

    void SetLogFrameConvention(FRAME_CONVENTION fc);


   private:
    bool Has(std::shared_ptr<FrLoggableBase> obj) const;

    std::string InitializeLogFolder();


   private:
    std::string m_log_folder;

    std::list<std::shared_ptr<FrLoggableBase>> m_loggable_list;

    FrOffshoreSystem* m_system;


  };

}  // end namespace frydom



#endif //FRYDOM_FRLOGMANAGER_H
