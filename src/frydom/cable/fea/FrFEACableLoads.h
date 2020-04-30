//
// Created by frongere on 15/04/2020.
//

#ifndef FRYDOM_FRFEACABLELOADS_H
#define FRYDOM_FRFEACABLELOADS_H

#include <chrono/physics/ChLoaderU.h>
#include <chrono/physics/ChLoad.h>
#include "FrFEACableElement.h"


namespace frydom {

  class FrOffshoreSystem;

  class FrFEACable;

  namespace internal {

    class FrFEAloaderBase : public chrono::ChLoaderUdistributed {

     public:
      FrFEAloaderBase(std::shared_ptr<chrono::ChLoadableU> loadable);

      int GetIntegrationPointsU() override;

      void SetNbIntegrationPoints(int n);

      void SetSystem(FrOffshoreSystem *system);

     protected:
      FrOffshoreSystem *m_system;

     private:
      int m_nb_integration_points;

    };


    class FrFEACableHydroLoader : public FrFEAloaderBase {

     public:

      explicit FrFEACableHydroLoader(std::shared_ptr<chrono::ChLoadableU> loadable);

      void ComputeF(const double U,
                    chrono::ChVectorDynamic<> &F,
                    chrono::ChVectorDynamic<> *state_x,
                    chrono::ChVectorDynamic<> *state_w) override;

      void SetCable(FrFEACable *cable);

     protected:
      FrFEACable *m_cable;

    };


    class FrFEACableHydroLoad : public chrono::ChLoad<FrFEACableHydroLoader> {
     public:

      FrFEACableHydroLoad(std::shared_ptr<chrono::ChLoadableU> loadable) :
          chrono::ChLoad<FrFEACableHydroLoader>(loadable) {}

      void SetCable(FrFEACable *cable) {
        loader.SetCable(cable);
      }

    };


  }  // end namespace frydom::internal









////  class FrOffshoreSystem;
//
//  class FrFEACable;
//
//
//
//  class FrFEACableLoader : public chrono::ChLoaderUdistributed {
//
//   public:
//    explicit FrFEACableLoader(std::shared_ptr<chrono::ChLoadableU> mloadable);
//
//    void SetCable(FrFEACable* cable);
//
//    void SetNbIntegrationPoints(const unsigned int n);
//
//    int GetIntegrationPointsU() override;
//
//   protected:
//    FrFEACable* m_cable;
//    FrOffshoreSystem* m_system;
//    int m_nb_integration_points;
//  };
//
//  class FrBuoyancyLoader : public FrFEACableLoader {
//
//   public:
//    explicit FrBuoyancyLoader(std::shared_ptr<chrono::ChLoadableU> mloadable);
//
//    void ComputeF(const double U,               ///< parametric coordinate in line
//                  chrono::ChVectorDynamic<> &F,         ///< Result F vector here, size must be = n.field coords.of loadable
//                  chrono::ChVectorDynamic<> *state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
//                  chrono::ChVectorDynamic<> *state_w) override;
//  };
//
//  class FrBuoyancyLoad : public chrono::ChLoad<FrBuoyancyLoader> {
//   public:
//    FrBuoyancyLoad(FrFEACable* cable, std::shared_ptr<chrono::ChLoadableU> mloadable);
//  };


}  // end namespace frydom



#endif //FRYDOM_FRFEACABLELOADS_H
