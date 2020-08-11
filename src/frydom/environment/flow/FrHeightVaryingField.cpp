//
// Created by lletourn on 10/08/20.
//

#include "FrHeightVaryingField.h"

frydom::Velocity frydom::FrHeightVaryingField::GetFluxVelocityInWorld(const frydom::Position &worldPos,
                                                                      frydom::FRAME_CONVENTION fc) const {
  auto height = worldPos.GetZ();
  if (IsNED(fc)) height = - height;
  Velocity velocity = m_direction * m_fieldInterpolator(height);
  if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(velocity);
  return velocity;
}

void frydom::FrHeightVaryingField::Set(std::vector<double> heights, std::vector<double> velocities, SPEED_UNIT speedUnit,
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
    std::vector<std::pair<double,double>> zipped(heightsTmp.size());
    for( size_t i = 0; i < heights.size(); i++ ) zipped[i] = std::make_pair( velocitiesTmp[i], heightsTmp[i] );

    // sort according to B
    std::sort(zipped.begin(), zipped.end(), []( auto & lop, auto & rop ) { return lop.second < rop.second; });

    // extract sorted A
    velocitiesTmp.clear();
    std::transform(zipped.begin(), zipped.end(), std::back_inserter(velocitiesTmp), []( auto & pair ){ return pair.first; });

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
