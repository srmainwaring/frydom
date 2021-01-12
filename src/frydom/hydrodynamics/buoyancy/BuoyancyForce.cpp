//
// Created by camille on 25/02/2020.
//

#include "BuoyancyForce.h"

namespace frydom {

  BuoyancyForce::BuoyancyForce(const std::string &name, FrBody *body, double volume, Position pos)
  : FrForce(name, "BuoyancyForce", body), m_volume(volume), m_pos(pos) { }

  void BuoyancyForce::Compute(double time) {
    auto rho_water = GetSystem()->GetEnvironment()->GetOcean()->GetDensity();
    auto gravity = GetSystem()->GetGravityAcceleration();
    SetForceInWorldAtPointInBody(Force(0., 0., m_volume*rho_water*gravity), m_pos, NWU);
  }

} // end namespace frydom