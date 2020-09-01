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

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;

TEST(FrHeightVaryingField, SetAndGet) {

  FRAME_CONVENTION fc = NWU;

  DIRECTION_CONVENTION dc = GOTO;

  FrHeightVaryingField field;

  Direction direction(1,0,0);
  direction.setRandom();
  direction.normalize();

  Eigen::VectorXd height(10), velocity(10);
  height.setRandom();
  height = 30*height;
  std::sort(height.data(), height.data() + height.size());
  velocity.setRandom();

//  std::cout<<"heights : "<<height<<std::endl;
//  std::cout<<"velocities : "<<velocity<<std::endl;

  std::vector<double> heights(&height[0], height.data()+height.cols()*height.rows());
  std::vector<double> velocities(&velocity[0], velocity.data()+velocity.cols()*velocity.rows());

  field.Set(heights, velocities, SPEED_UNIT::MS, direction, fc, dc);

//  std::cout<<"velocities out : ";
//  for (auto& item:heights) std::cout<<field.GetFluxVelocityInWorld(Position(0,0,item), fc)<<", ";
//  std::cout<<std::endl;

  for (int i=0; i<heights.size();i++) {
    auto dirTmp = direction;
    if (IsNED(fc)) internal::SwapFrameConvention(dirTmp);
    if (IsCOMEFROM(dc)) dirTmp = -dirTmp;
    double velocityOut = field.GetFluxVelocityInWorld(Position(0,0,heights[i]), fc).transpose() * dirTmp;
    EXPECT_FLOAT_EQ(velocities[i], velocityOut);
  }
}