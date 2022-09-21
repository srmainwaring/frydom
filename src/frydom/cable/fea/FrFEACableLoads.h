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

     private:
      int m_nb_integration_points;

     public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };


    class FrFEACableHydroLoader : public FrFEAloaderBase {

     public:

      explicit FrFEACableHydroLoader(std::shared_ptr<chrono::ChLoadableU> loadable);

      void ComputeF_(const double U,
                    chrono::ChVectorDynamic<> &F,
                    chrono::ChVectorDynamic<> *state_x,
                    chrono::ChVectorDynamic<> *state_w); // TODO: supprimer

      void ComputeF(const double U,
                    chrono::ChVectorDynamic<> &F,
                    chrono::ChVectorDynamic<> *state_x,
                    chrono::ChVectorDynamic<> *state_w) override;

      void SetCable(FrFEACable *cable);

//     private:
//      static inline double fn(const double &alpha) {
//        double twice_alpha = 2. * alpha;
//        return 0.5 - 0.1 * std::cos(alpha) + 0.1 * std::sin(alpha) - 0.4 * cos(twice_alpha) - 0.11 * sin(twice_alpha);
////        return 1.; // TODO: supprimer
//      }
//
//      static inline double ft(const double &alpha) {
//        double alpha2 = alpha * alpha;
//        double alpha3 = alpha2 * alpha;
//        double alpha4 = alpha3 * alpha;
//        double alpha5 = alpha4 * alpha;
//        return 0.01 *
//               (2.008 - 0.3858 * alpha + 1.9159 * alpha2 - 4.16147 * alpha3 + 3.5064 * alpha4 - 1.187299 * alpha5);
////        return 1.; // TODO: supprimer
//      }

     protected:
      FrFEACable *m_cable;

     public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };


    class FrFEACableHydroLoad : public chrono::ChLoad<FrFEACableHydroLoader> {
     public:

      FrFEACableHydroLoad(std::shared_ptr<chrono::ChLoadableU> loadable) :
          chrono::ChLoad<FrFEACableHydroLoader>(loadable) {}

      void SetCable(FrFEACable *cable) {
        loader.SetCable(cable);
      }

     public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
