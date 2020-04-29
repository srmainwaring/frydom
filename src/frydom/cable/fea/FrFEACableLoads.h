//
// Created by frongere on 15/04/2020.
//

#ifndef FRYDOM_FRFEACABLELOADS_H
#define FRYDOM_FRFEACABLELOADS_H

#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChLoaderU.h"
#include "chrono/core/ChVectorDynamic.h"
#include "chrono/fea/ChElementBeamEuler.h"

//#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/environment/FrEnvironmentInc.h"

namespace frydom {

//  class FrOffshoreSystem;

  class FrFEACable;



  class FrFEACableLoader : public chrono::ChLoaderUdistributed {

   public:
    explicit FrFEACableLoader(std::shared_ptr<chrono::ChLoadableU> mloadable);

    void SetCable(FrFEACable* cable);

    void SetNbIntegrationPoints(const unsigned int n);

    int GetIntegrationPointsU() override;

   protected:
    FrFEACable* m_cable;
    FrOffshoreSystem* m_system;
    int m_nb_integration_points;
  };

  class FrBuoyancyLoader : public FrFEACableLoader {

   public:
    explicit FrBuoyancyLoader(std::shared_ptr<chrono::ChLoadableU> mloadable);

    void ComputeF(const double U,               ///< parametric coordinate in line
                  chrono::ChVectorDynamic<> &F,         ///< Result F vector here, size must be = n.field coords.of loadable
                  chrono::ChVectorDynamic<> *state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                  chrono::ChVectorDynamic<> *state_w) override;
  };

  class FrBuoyancyLoad : public chrono::ChLoad<FrBuoyancyLoader> {
   public:
    FrBuoyancyLoad(FrFEACable* cable, std::shared_ptr<chrono::ChLoadableU> mloadable);
  };


}  // end namespace frydom



#endif //FRYDOM_FRFEACABLELOADS_H
