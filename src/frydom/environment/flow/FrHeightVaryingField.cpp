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

#include "FrHeightVaryingField.h"

namespace frydom {

  Velocity frydom::FrHeightVaryingField::GetFluxVelocityInWorld(const frydom::Position &worldPos,
                                                                        frydom::FRAME_CONVENTION fc) const {
    auto height = worldPos.GetZ();
    if (IsNED(fc)) height = -height;
    Velocity velocity = m_direction * m_fieldInterpolator(height);
    if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(velocity);
    return velocity;
  }

  void
  FrHeightVaryingField::Set(std::vector<double> heights, std::vector<double> velocities, SPEED_UNIT speedUnit,
                                    frydom::Direction direction, frydom::FRAME_CONVENTION fc,
                                    frydom::DIRECTION_CONVENTION dc) {

    auto dirTmp = direction;
    auto velocitiesTmp = velocities;
    auto heightsTmp = heights;


    if (IsNED(fc)) {
      internal::SwapFrameConvention<Velocity>(dirTmp);
      for (auto &item:heightsTmp) item = -item;

      //TODO:: should we really sort?

      // Sorting the velocities vector according to the heights vector
      // zip the two vectors (A,B)
      std::vector<std::pair<double, double>> zipped(heightsTmp.size());
      for (size_t i = 0; i < heights.size(); i++) zipped[i] = std::make_pair(velocitiesTmp[i], heightsTmp[i]);

      // sort according to B
      std::sort(zipped.begin(), zipped.end(), [](auto &lop, auto &rop) { return lop.second < rop.second; });

      // extract sorted A
      velocitiesTmp.clear();
      std::transform(zipped.begin(), zipped.end(), std::back_inserter(velocitiesTmp),
                     [](auto &pair) { return pair.first; });

      // sort the heights
      std::sort(heightsTmp.begin(), heightsTmp.end());

    }

    if (IsCOMEFROM(dc)) {
      dirTmp = -dirTmp;
    }

    for (auto &item:velocitiesTmp) item = mathutils::convert_velocity_unit(item, speedUnit, mathutils::MS);

    m_direction = dirTmp;

    auto heightsIn = std::make_shared<const std::vector<double>>(heightsTmp);
    auto velocitiesIn = std::make_shared<const std::vector<double>>(velocitiesTmp);

    m_fieldInterpolator.Initialize(heightsIn, velocitiesIn);

  }

  void FrHeightVaryingField::Set(std::vector<double> heights, std::vector<double> velocities, SPEED_UNIT speedUnit,
                                 double angle, ANGLE_UNIT angleUnit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
    if (angleUnit == mathutils::DEG) angle *= DEG2RAD; // Convert in RAD
    Set(heights, velocities, speedUnit, {cos(angle), sin(angle), 0}, fc, dc);
  }

} // end namespace frydom