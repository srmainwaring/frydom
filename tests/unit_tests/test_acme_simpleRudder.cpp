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

template<typename T>
std::string str(T begin, T end) {
  std::stringstream ss;
  bool first = true;
  ss << "[";
  for (; begin != end; begin++) {
    if (!first)
      ss << ", ";
    ss << *begin;
    first = false;
  }
  ss << "]";
  return ss.str();
}


TEST(TestRudder, without_interactions) {


  std::vector<double> flow_incidence_on_main_rudder_deg = {-28, -26, -24.0, -22.0, -20.0, -18.0, -16.0, -14.0, -12.0,
                                                           -10.0, -8.0, -6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0,
                                                           12.0, 14.0, 16.0, 18.0, 20.0, 22.0, 24.0, 26.0, 28.0};
  // FIXME all cd should be negative...
  std::vector<double> cd = {0.2778935134410858, 0.24202793836593628, 0.18181930482387543, 0.09083189815282822,
                            0.04477076604962349, 0.026631886139512062, 0.019129594787955284, 0.015245308168232441,
                            0.012423327192664146, 0.01028597541153431, 0.008432770147919655, 0.007111922837793827,
                            0.006130721885710955, 0.00546258594840765, 0.005242994520813227, 0.00546258594840765,
                            0.006130721885710955, 0.007111922837793827, 0.008432770147919655, 0.01028597541153431,
                            0.012423327192664146, 0.015245308168232441, 0.019129594787955284, 0.026631886139512062,
                            0.04477076604962349, 0.09083189815282822, 0.18181930482387543, 0.24202793836593628,
                            0.2778935134410858};
  std::vector<double> cl = {-1.1561044454574585, -1.1501551866531372,
                            -1.2516950368881226, -1.5970512628555298, -1.742898941040039, -1.7230695486068726,
                            -1.6346718072891235, -1.4956920146942139, -1.316851258277893, -1.1179311275482178,
                            -0.8795303702354431, -0.6680029630661011, -0.4484163820743561, -0.2255769968032837,
                            2.668157685548067e-05, 0.2255769968032837, 0.4484163820743561, 0.6680029630661011,
                            0.8795303702354431, 1.1179311275482178, 1.316851258277893, 1.4956920146942139,
                            1.6346718072891235, 1.7230695486068726, 1.742898941040039, 1.5970512628555298,
                            1.2516950368881226, 1.1501551866531372, 1.1561044454574585};
  std::vector<double> cn = {0.11684787273406982, 0.08721555024385452, 0.04257682338356972, -0.011721551418304443,
                            -0.03142339736223221, -0.03119639679789543, -0.0205577053129673, -0.009575597010552883,
                            -0.0033826581202447414, 0.00010325611947337165, -0.004063922446221113,
                            -0.0013044830411672592, -0.00022130433353595436, 0.00012940994929522276,
                            -3.3430785606469726e-06, -0.00012940994929522276, 0.00022130433353595436,
                            0.0013044830411672592, 0.004063922446221113, -0.00010325611947337165,
                            0.0033826581202447414, 0.009575597010552883, 0.0205577053129673, 0.03119639679789543,
                            0.03142339736223221, 0.011721551418304443, -0.04257682338356972, -0.08721555024385452,
                            -0.11684787273406982};

  std::stringstream ss;
  ss << R"({"flow_incidence_on_main_rudder_deg": )"
     << str(flow_incidence_on_main_rudder_deg.begin(), flow_incidence_on_main_rudder_deg.end())
     << R"(, "Cd": )" << str(cd.begin(), cd.end())
     << R"(, "Cl": )" << str(cl.begin(), cl.end())
     << R"(, "Cn": )" << str(cn.begin(), cn.end())
     << R"(, "frame_convention": "NWU", "direction_convention": "COMEFROM")"
     << "}";


  acme::RudderParams params;
  params.m_hull_wake_fraction_0 = 0.;
  params.m_chord_m = 2.;
  params.m_lateral_area_m2 = 4.;
  params.m_flap_slope = 0.; //NA

  auto acme_rudder = SimpleRudderModel(params, ss.str());
  acme_rudder.Initialize();

  // u = 0, v = 0
  acme_rudder.Compute(1025, 0., 0., 0.);
  EXPECT_EQ(acme_rudder.GetFx(), 0.);
  EXPECT_EQ(acme_rudder.GetFy(), 0.);
  EXPECT_EQ(acme_rudder.GetMz(), 0.);

  // u = 1, v = 0, delta = 0
  acme_rudder.Compute(1025, 1., 0., 0.);
  EXPECT_NEAR(acme_rudder.GetFx(), 10.7481, 1E-4);
  EXPECT_NEAR(acme_rudder.GetFy(), 0.0546973, 1E-7);
  EXPECT_NEAR(acme_rudder.GetMz(), -0.0137066, 1E-7);

  // u = 1, v = 0, delta = 20
  acme_rudder.Compute(1025, 1., 0., 20.);
  EXPECT_NEAR(acme_rudder.GetFx(), 91.7801, 1E-4);
  EXPECT_NEAR(acme_rudder.GetFy(), 3572.94, 1E-2);
  EXPECT_NEAR(acme_rudder.GetMz(), 128.836, 1E-3);

  // u = 1, v = 0, delta = -20
  acme_rudder.Compute(1025, 1., 0., -20.);
  EXPECT_NEAR(acme_rudder.GetFx(), 91.7801, 1E-4);
  EXPECT_NEAR(acme_rudder.GetFy(), -3572.94, 1E-2);
  EXPECT_NEAR(acme_rudder.GetMz(), -128.836, 1E-3);

  // u = 2, v = 0, delta = 20
  acme_rudder.Compute(1025, 2., 0., 20.);
  EXPECT_NEAR(acme_rudder.GetFx(), 367.121, 1E-3);
  EXPECT_NEAR(acme_rudder.GetFy(), 14291.8, 1E-1);
  EXPECT_NEAR(acme_rudder.GetMz(), 515.344, 1E-3);

  // Intepolators evaluated outside of their range
  EXPECT_EXIT(acme_rudder.Compute(1025, 1., 0., 30), testing::ExitedWithCode(1),"");

  // u = cos(-20), v = sin(-20), delta = 0
  acme_rudder.Compute(1025, std::cos(-20*DEG2RAD), std::sin(-20*DEG2RAD), 0.);
  EXPECT_NEAR(acme_rudder.GetFx(), 1308.26, 1E-2);
  EXPECT_NEAR(acme_rudder.GetFy(), 3326.08, 1E-2);
  EXPECT_NEAR(acme_rudder.GetMz(), 128.836, 1E-3);

  // with different parameters
  params.m_hull_wake_fraction_0 = 0.;
  params.m_chord_m = 3.;
  params.m_lateral_area_m2 = 15.;
  params.m_flap_slope = 0.; //NA

  auto acme_rudder2 = SimpleRudderModel(params, ss.str());
  acme_rudder2.Initialize();

  // u = 1, v = 0, delta = 20
  acme_rudder2.Compute(1025, 1., 0., 20.);
  EXPECT_NEAR(acme_rudder2.GetFx(), 344.176, 1E-3);
  EXPECT_NEAR(acme_rudder2.GetFy(), 13398.5, 1E-1);
  EXPECT_NEAR(acme_rudder2.GetMz(), 724.702, 1E-3);

  // with hull/rudder interactions
  params.m_hull_wake_fraction_0 = 0.2;
  params.m_chord_m = 2.;
  params.m_lateral_area_m2 = 4.;
  params.m_flap_slope = 0.; //NA

  auto acme_rudder3 = SimpleRudderModel(params, ss.str());
  acme_rudder3.Initialize();

  // u = 1, v = 0, delta = 0
  acme_rudder3.Compute(1025, 1., 0., 0.);
  EXPECT_NEAR(acme_rudder3.GetFx(), 6.8788, 1E-4);
  EXPECT_NEAR(acme_rudder3.GetFy(), 0.0350063, 1E-7);
  EXPECT_NEAR(acme_rudder3.GetMz(), -0.00877224, 1E-8);

  // u = 1, v = 0, delta = 20
  acme_rudder3.Compute(1025, 1., 0., 20.);
  EXPECT_NEAR(acme_rudder3.GetFx(), 58.7393, 1E-4);
  EXPECT_NEAR(acme_rudder3.GetFy(), 2286.68, 1E-2);
  EXPECT_NEAR(acme_rudder3.GetMz(), 82.455, 1E-3);

  // u = 2, v = 0, delta = 20
  acme_rudder3.Compute(1025, 2., 0., 20.);
  EXPECT_NEAR(acme_rudder3.GetFx(), 234.957, 1E-3);
  EXPECT_NEAR(acme_rudder3.GetFy(), 9146.74, 1E-2);
  EXPECT_NEAR(acme_rudder3.GetMz(), 329.82, 1E-2);

  // u = cos(-20), v = sin(-20), delta = 0
  acme_rudder3.Compute(1025, std::cos(-20*DEG2RAD), std::sin(-20*DEG2RAD), 0.);
  EXPECT_NEAR(acme_rudder3.GetFx(), 1115.11, 1E-2);
  EXPECT_NEAR(acme_rudder3.GetFy(), 2196.63, 1E-2);
  EXPECT_NEAR(acme_rudder3.GetMz(), -9.21777, 1E-5);

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}