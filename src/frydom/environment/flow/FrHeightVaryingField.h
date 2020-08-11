//
// Created by lletourn on 10/08/20.
//

#ifndef FRYDOM_FRHEIGHTVARYINGFIELD_H
#define FRYDOM_FRHEIGHTVARYINGFIELD_H

#include "FrFieldBase.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrUnits.h"

#include "MathUtils/Interp1d.h"


namespace frydom {

  /**
  * \class FrHeightVaryingField
  * \brief Class for defining a height varying field (current or wind).
  */
  class FrHeightVaryingField : public FrFieldBase {

   public:

    /// Return the flow velocity at a given point in world frame
    /// \param worldPos Position of the Point in world frame
    /// \param fc Frame convention (NED/NWU)
    /// \return Velocity in world frame
    Velocity GetFluxVelocityInWorld(const Position &worldPos, FRAME_CONVENTION fc) const override;

    /// Update the state of the field model (virtual pure)
    /// \param time Current time of the simulation
    void Update(double time) override {};

    void Set(std::vector<double> heights, std::vector<double> velocities, SPEED_UNIT speedUnit,
             Direction direction, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);

   private:

    mathutils::Interp1dLinear<double, double> m_fieldInterpolator;

    mathutils::Vector3d<double> m_direction;


    };


} // end namespace frydom


#endif //FRYDOM_FRHEIGHTVARYINGFIELD_H
