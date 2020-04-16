//
// Created by frongere on 15/04/2020.
//

#ifndef FRYDOM_FRHYDROFEACABLELOADS_H
#define FRYDOM_FRHYDROFEACABLELOADS_H

#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChLoaderU.h"
#include "chrono/core/ChVectorDynamic.h"
#include "chrono/fea/ChElementBeamEuler.h"

//#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/environment/FrEnvironmentInc.h"

namespace frydom {

//  class FrOffshoreSystem;

  class FrDynamicCable;







  class FrHydroFEACableLoader : public chrono::ChLoaderUdistributed {

   public:
    explicit FrHydroFEACableLoader(std::shared_ptr<chrono::ChLoadableU> mloadable);

    void SetSystem(FrOffshoreSystem *system);

    int GetIntegrationPointsU() override;

   protected:
    FrOffshoreSystem *m_system;

  };

  class FrBuoyancyLoader : public FrHydroFEACableLoader {

   public:
    explicit FrBuoyancyLoader(std::shared_ptr<chrono::ChLoadableU> mloadable);

    void ComputeF(const double U,               ///< parametric coordinate in line
                  chrono::ChVectorDynamic<> &F,         ///< Result F vector here, size must be = n.field coords.of loadable
                  chrono::ChVectorDynamic<> *state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                  chrono::ChVectorDynamic<> *state_w) override;
  };

  class FrBuoyancyLoad : public chrono::ChLoad<FrBuoyancyLoader> {
   public:
    FrBuoyancyLoad(FrOffshoreSystem *system, std::shared_ptr<chrono::ChLoadableU> mloadable);
  };


}  // end namespace frydom



#endif //FRYDOM_FRHYDROFEACABLELOADS_H
