//
// Created by frongere on 15/04/2020.
//

#include "FrFEACableLoads.h"


namespace frydom {


  FrFEACableLoader::FrFEACableLoader(std::shared_ptr<chrono::ChLoadableU> mloadable) :
      m_system(nullptr), chrono::ChLoaderUdistributed(mloadable) {}

  void FrFEACableLoader::SetSystem(FrOffshoreSystem *system) {
    m_system = system;
  }

  int FrFEACableLoader::GetIntegrationPointsU() { return 1; }

  FrBuoyancyLoader::FrBuoyancyLoader(std::shared_ptr<chrono::ChLoadableU> mloadable) :
      FrFEACableLoader(mloadable) {}

  void FrBuoyancyLoader::ComputeF(const double U, chrono::ChVectorDynamic<> &F, chrono::ChVectorDynamic<> *state_x,
                                  chrono::ChVectorDynamic<> *state_w) {  ///< if != 0, update state (speed part) to this, then evaluate F)

    // Pas besoin d'agir sur state_x et state_w

    // FIXME: il faudra templater cet element !!
    auto element = std::dynamic_pointer_cast<chrono::fea::ChElementBeamEuler>(loadable);


    // Get the point position
    double eta = U;
    chrono::ChVector<double> position_;
    chrono::ChQuaternion<double> rotation_;

    element->EvaluateSectionFrame(eta,position_,rotation_);

    auto position = internal::ChVectorToVector3d<Position>(position_);


    double gravity = m_system->GetGravityAcceleration();
    // Voir si on met pas false pour la prise en compte de la deformee de la surface libre
    auto fluid_type = m_system->GetEnvironment()->GetFluidTypeAtPointInWorld(position, NWU,true);
    auto fluid_density = m_system->GetEnvironment()->GetFluidDensity(fluid_type);

//      double diameter = m_cable->GetCableProperties()->GetDiameter();
//      chrono::ChVector<double> unit_force = {0., 0., fluid_density * diameter};




    // Ce qui est fait dans ChLoaderBeamWrenchDistributed
//      F.PasteVector(unit_force, 0, 0);   // load, force part
    F.PasteVector(chrono::VNULL, 3, 0);  // No torque


  }

  FrBuoyancyLoad::FrBuoyancyLoad(FrOffshoreSystem *system, std::shared_ptr<chrono::ChLoadableU> mloadable) :
      chrono::ChLoad<FrBuoyancyLoader>(mloadable) {
    loader.SetSystem(system);
  }
}  // end namespace frydom
