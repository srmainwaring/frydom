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

#include "acme/acme.h"
#include "gtest/gtest.h"

using namespace acme;


TEST(TestFPP4Q, without_interactions) {

  std::string open_water_data_table = R"({"beta_deg": [-180.0,-140.0,-100.0,-60.00000000000001,-20.000000000000004,20.000000000000004,59.999999999999986,99.99999999999999,140.0,180.0],
      "ct": [-0.16268168999999996,0.4486688377385315,0.8944023203183783,0.8032479305281882,0.27572401377867956,0.017939934846272564,-1.142724540528188,-1.3839567584729406,-0.7788844482089202,-0.16268169000000005],
      "cq": [-0.018609981000000005,0.055124843060058874,0.10925670286110463,0.1255048384078987,0.05556630922628392,0.015411367752487179,-0.10401960640789869,-0.12289300851577142,-0.0810090123841631,-0.018609980999999987]
    })";

  ThrusterBaseParams params = {2., 0., 0., SCREW_DIRECTION::RIGHT_HANDED};
  FPP4Q propeller(params, open_water_data_table);
  propeller.Initialize();

  // beta = 0, n = 0
  propeller.Compute(1025, 0., 0., 0., 0.);
  EXPECT_EQ(propeller.GetThrust(), 0.);
  EXPECT_EQ(propeller.GetTorque(), 0.);
  EXPECT_EQ(propeller.GetPower(), 0.);
  EXPECT_EQ(propeller.GetPropellerEfficiency(), 0.);

  // beta = 0, n = 1
  propeller.Compute(1025, 0., 0., 60., 0.);
  EXPECT_NEAR(propeller.GetThrust(), 4573.2, 1E-1);
  EXPECT_NEAR(propeller.GetTorque(), 2210.66, 1E-2);
  EXPECT_NEAR(propeller.GetPower(), 13890, 1E-0);
  EXPECT_EQ(propeller.GetPropellerEfficiency(), 0.);

  // beta = + Pi/2, n = 0
  propeller.Compute(1025, 1., 0., 0., 0.);
  EXPECT_NEAR(propeller.GetThrust(), -2131.16, 1E-2);
  EXPECT_NEAR(propeller.GetTorque(), -380.538, 1E-3);
  EXPECT_EQ(propeller.GetPower(), 0.);
  EXPECT_EQ(propeller.GetPropellerEfficiency(), 0);

  // beta = - Pi/2, n = 0
  propeller.Compute(1025, -1., 0., 0., 0.);
  EXPECT_NEAR(propeller.GetThrust(), 1403.36, 1E-2);
  EXPECT_NEAR(propeller.GetTorque(), 364.901, 1E-3);
  EXPECT_EQ(propeller.GetPower(), 0.);
  EXPECT_EQ(propeller.GetPropellerEfficiency(), 0);

  // beta = 0.223564 rad
  propeller.Compute(1025, 1., 0., 60, 0.);
  EXPECT_NEAR(propeller.GetThrust(), 2105.6, 1E-1);
  EXPECT_NEAR(propeller.GetTorque(), 1482.53, 1E-2);
  EXPECT_NEAR(propeller.GetPower(), 9315, 1E-0);
  EXPECT_NEAR(propeller.GetPropellerEfficiency(), 0.226043, 1E-6);

  // beta = -0.223564 rad
  propeller.Compute(1025, -1., 0., 60, 0.);
  EXPECT_NEAR(propeller.GetThrust(), 7513.62, 1E-2);
  EXPECT_NEAR(propeller.GetTorque(), 3167.34, 1E-2);
  EXPECT_NEAR(propeller.GetPower(), 19901, 1E-0);
  EXPECT_NEAR(propeller.GetPropellerEfficiency(), -0.37755, 1E-5);

  // beta = 2.91803 rad
  propeller.Compute(1025, 1., 0., -60, 0.);
  EXPECT_NEAR(propeller.GetThrust(), -11792.4, 1E-1);
  EXPECT_NEAR(propeller.GetTorque(), -2528.24, 1E-2);
  EXPECT_NEAR(propeller.GetPower(), 15885.4, 1E-1);
  EXPECT_NEAR(propeller.GetPropellerEfficiency(), -0.742344, 1E-6);

  // beta = -2.91803 rad
  propeller.Compute(1025, -1., 0., -60, 0.);
  EXPECT_NEAR(propeller.GetThrust(), 1083.95, 1E-2);
  EXPECT_NEAR(propeller.GetTorque(), 327.704, 1E-3);
  EXPECT_NEAR(propeller.GetPower(), -2059.02, 1E-2);
  EXPECT_NEAR(propeller.GetPropellerEfficiency(), 0.52644, 1E-5);

  // Lateral flow velocity
  propeller.Compute(1025, 0, 1, 60, 0.);
  EXPECT_NEAR(propeller.GetThrust(), 4573.2, 1E-1);
  EXPECT_NEAR(propeller.GetTorque(), 2210.66, 1E-2);
  EXPECT_NEAR(propeller.GetPower(), 13890, 1E-0);
  EXPECT_EQ(propeller.GetPropellerEfficiency(), 0.);

  // 20 degree side wash angle
  propeller.Compute(1025, std::cos(20*DEG2RAD), std::sin(20*DEG2RAD), 60, 0.);
  EXPECT_NEAR(propeller.GetThrust(), 2250.73, 1E-2);
  EXPECT_NEAR(propeller.GetTorque(), 1522.99, 1E-2);
  EXPECT_NEAR(propeller.GetPower(), 9569.24, 1E-2);
  EXPECT_NEAR(propeller.GetPropellerEfficiency(), 0.22102, 1E-6);

  // Double the rpm
  propeller.Compute(1025, std::cos(20*DEG2RAD), std::sin(20*DEG2RAD), 120, 0.);
  EXPECT_NEAR(propeller.GetThrust(), 13550, 1E-0);
  EXPECT_NEAR(propeller.GetTorque(), 7400.94, 1E-2);
  EXPECT_NEAR(propeller.GetPower(), 93002.9, 1E-1);
  EXPECT_NEAR(propeller.GetPropellerEfficiency(), 0.136908, 1E-6);
}

TEST(TestFPP4Q, with_interactions) {

  std::string open_water_data_table = R"({"beta_deg": [-180.0,-140.0,-100.0,-60.00000000000001,-20.000000000000004,20.000000000000004,59.999999999999986,99.99999999999999,140.0,180.0],
      "ct": [-0.16268168999999996,0.4486688377385315,0.8944023203183783,0.8032479305281882,0.27572401377867956,0.017939934846272564,-1.142724540528188,-1.3839567584729406,-0.7788844482089202,-0.16268169000000005],
      "cq": [-0.018609981000000005,0.055124843060058874,0.10925670286110463,0.1255048384078987,0.05556630922628392,0.015411367752487179,-0.10401960640789869,-0.12289300851577142,-0.0810090123841631,-0.018609980999999987]
    })";

  ThrusterBaseParams params = {2., 0.2, 0.25, SCREW_DIRECTION::RIGHT_HANDED};
  FPP4Q propeller(params, open_water_data_table);
  propeller.Initialize();

  // beta = 0, n = 0
  propeller.Compute(1025, 0., 0., 0., 0.);
  EXPECT_EQ(propeller.GetThrust(), 0.);
  EXPECT_EQ(propeller.GetTorque(), 0.);
  EXPECT_EQ(propeller.GetPower(), 0.);
  EXPECT_EQ(propeller.GetPropellerEfficiency(), 0.);

  // beta = 0, n = 1
  propeller.Compute(1025, 0., 0., 60., 0.);
  EXPECT_NEAR(propeller.GetThrust(), 3429.9, 1E-1);
  EXPECT_NEAR(propeller.GetTorque(), 2210.66, 1E-2);
  EXPECT_NEAR(propeller.GetPower(), 13890, 1E-0);
  EXPECT_EQ(propeller.GetPropellerEfficiency(), 0.);

  // beta = + Pi/2, n = 0
  propeller.Compute(1025, 1., 0., 0., 0.);
  EXPECT_NEAR(propeller.GetThrust(), -1022.96, 1E-2);
  EXPECT_NEAR(propeller.GetTorque(), -243.544, 1E-3);
  EXPECT_EQ(propeller.GetPower(), 0.);
  EXPECT_EQ(propeller.GetPropellerEfficiency(), 0);

  // beta = - Pi/2, n = 0
  propeller.Compute(1025, -1., 0., 0., 0.);
  EXPECT_NEAR(propeller.GetThrust(), 1052.52, 1E-2);
  EXPECT_NEAR(propeller.GetTorque(), 364.901, 1E-3);
  EXPECT_EQ(propeller.GetPower(), 0.);
  EXPECT_EQ(propeller.GetPropellerEfficiency(), 0);

  // beta = 0.179924 rad
  propeller.Compute(1025, 1., 0., 60, 0.);
  EXPECT_NEAR(propeller.GetThrust(), 1940.11, 1E-2);
  EXPECT_NEAR(propeller.GetTorque(), 1617.82, 1E-2);
  EXPECT_NEAR(propeller.GetPower(), 10165.1, 1E-1);
  EXPECT_NEAR(propeller.GetPropellerEfficiency(), 0.203585, 1E-6);

  // beta = -0.223564 rad
  propeller.Compute(1025, -1., 0., 60, 0.);
  EXPECT_NEAR(propeller.GetThrust(), 5635.22, 1E-2);
  EXPECT_NEAR(propeller.GetTorque(), 3167.34, 1E-2);
  EXPECT_NEAR(propeller.GetPower(), 19901, 1E-0);
  EXPECT_NEAR(propeller.GetPropellerEfficiency(), -0.37755, 1E-5);

  // beta = 2.96167 rad
  propeller.Compute(1025, 1., 0., -60, 0.);
  EXPECT_NEAR(propeller.GetThrust(), -7758.28, 1E-2);
  EXPECT_NEAR(propeller.GetTorque(), -2232.49, 1E-2);
  EXPECT_NEAR(propeller.GetPower(), 14027.2, 1E-1);
  EXPECT_NEAR(propeller.GetPropellerEfficiency(), -0.589963, 1E-6);

  // beta = -2.91803 rad
  propeller.Compute(1025, -1., 0., -60, 0.);
  EXPECT_NEAR(propeller.GetThrust(), 812.964, 1E-2);
  EXPECT_NEAR(propeller.GetTorque(), 327.704, 1E-3);
  EXPECT_NEAR(propeller.GetPower(), -2059.02, 1E-2);
  EXPECT_NEAR(propeller.GetPropellerEfficiency(), 0.52644, 1E-5);

  // Lateral flow velocity
  propeller.Compute(1025, 0, 1, 60, 0.);
  EXPECT_NEAR(propeller.GetThrust(), 3429.9, 1E-1);
  EXPECT_NEAR(propeller.GetTorque(), 2210.66, 1E-2);
  EXPECT_NEAR(propeller.GetPower(), 13890, 1E-0);
  EXPECT_EQ(propeller.GetPropellerEfficiency(), 0.);

  // 20 degree side wash angle _ beta = 0.185257 rad
  propeller.Compute(1025, std::cos(20*DEG2RAD), std::sin(20*DEG2RAD), 60, 0.);
  EXPECT_NEAR(propeller.GetThrust(), 1896.32, 1E-2);
  EXPECT_NEAR(propeller.GetTorque(), 1601.23, 1E-2);
  EXPECT_NEAR(propeller.GetPower(), 10060.8, 1E-1);
  EXPECT_NEAR(propeller.GetPropellerEfficiency(), 0.207147, 1E-6);

  // Double the rpm _ beta = 0.0934303 rad
  propeller.Compute(1025, std::cos(20*DEG2RAD), std::sin(20*DEG2RAD), 120, 0.);
  EXPECT_NEAR(propeller.GetThrust(), 10588.3, 1E-1);
  EXPECT_NEAR(propeller.GetTorque(), 7569.52, 1E-2);
  EXPECT_NEAR(propeller.GetPower(), 95121.4, 1E-1);
  EXPECT_NEAR(propeller.GetPropellerEfficiency(), 0.122334, 1E-6);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}