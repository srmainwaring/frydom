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

#include <string>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace frydom {

  Velocity frydom::FrHeightVaryingField::GetFluxVelocityInWorld(const frydom::Position &worldPos,
                                                                        frydom::FRAME_CONVENTION fc) const {
    auto height = worldPos.GetZ();
    if (IsNED(fc)) height = -height;
    Velocity velocity = m_direction * m_fieldInterpolator(height);
    if (IsNED(fc)) velocity = internal::SwapFrameConvention<Velocity>(velocity);
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
      dirTmp = internal::SwapFrameConvention<Velocity>(dirTmp);
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

  void FrHeightVaryingField::ReadJSON(const std::string &json_file) {
    std::ifstream ifs(json_file);
    auto json_obj = json::parse(ifs);

    if (json_obj.find("field") == json_obj.end()) {
      std::cout << "error : field is not defined in the input file :" << json_file << std::endl;
      ifs.close();
      exit(1);
    }

    auto param = json_obj["field"];

    auto angle = param.at("angle").get<double>();

    auto angle_unit_str = param.at("angle_unit").get<std::string>();

    ANGLE_UNIT angle_unit;
    if (angle_unit_str == "DEG") {
      angle_unit = mathutils::DEG;
    } else if (angle_unit_str == "RAD") {
      angle_unit = mathutils::RAD;
    } else {
      throw FrException("unknown value for the angle unit value");
    }

    SPEED_UNIT speed_unit;
    auto speed_unit_str = param.at("speed_unit").get<std::string>();
    if (speed_unit_str == "MS") {
      speed_unit = mathutils::MS;
    } else if (speed_unit_str == "KNOT") {
      speed_unit = mathutils::KNOT;
    } else if (speed_unit_str == "KMH") {
      speed_unit = mathutils::KMH;
    } else {
      throw FrException("unknown value for the speed unit value");
    }

    auto fc = param.at("frame_convention").get<std::string>();
    auto dc = param.at("direction_convention").get<std::string>();

    auto heights = param.at("heights").get<std::vector<double>>();
    auto velocities = param.at("velocities").get<std::vector<double>>();
    Set(heights, velocities, speed_unit, angle, angle_unit, STRING2FRAME(fc), STRING2DIRECTION(dc));

  }

} // end namespace frydom