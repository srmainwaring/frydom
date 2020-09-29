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

    /// Definition of the height varying field from direction and magnitudes
    /// \param heights heights for the definitions of the velocity profile
    /// \param velocities Velocities of the flow at corresponding heights
    /// \param speed_unit Speed unit (MS/KMH/KNOT)
    /// \param direction Direction of the flow
    /// \param fc Frame convention (NED/NWU)
    /// \param dc Direction convention (GOTO/COMEFROM)
    void Set(std::vector<double> heights, std::vector<double> velocities, SPEED_UNIT speedUnit,
             Direction direction, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);

    /// Definition of the height varying field from angle and magnitudes
    /// \param heights heights for the definitions of the velocity profile
    /// \param velocities Velocities of the flow at corresponding heights
    /// \param speedUnit Speed unit (MS/KMH/KNOT)
    /// \param angle Direction angle of the flow
    /// \param angleUnit Angle unit (RAD/DEG)
    /// \param fc Frame convention (NED/NWU)
    /// \param dc Direction convention (GOTO/COMEFROM)
    void Set(std::vector<double> heights, std::vector<double> velocities, SPEED_UNIT speedUnit,
        double angle, ANGLE_UNIT angleUnit,  FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);

    void ReadJSON(const std::string &json_file);

   private:

    mathutils::Interp1dLinear<double, double> m_fieldInterpolator;

    mathutils::Vector3d<double> m_direction;


    };


} // end namespace frydom


#endif //FRYDOM_FRHEIGHTVARYINGFIELD_H
