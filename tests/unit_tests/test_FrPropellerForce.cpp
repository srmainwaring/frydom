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

class testForce : public FrPropellerForce {

 public:

  testForce(const std::string& name, FrBody *body, Position propellerPositionInBody);

  double ComputeLongitudinalVelocity();

  double GetWakeFraction(double sidewashAngle) const;

  void ReadCoefficientsFile() override;

  void SetPitchRatio(double P_D) override {};

  double GetPitchRatio() const override {};

  GeneralizedForce ComputeGeneralizedForceInBody() override;

};

double testForce::ComputeLongitudinalVelocity() {
  return FrPropellerForce::ComputeLongitudinalVelocity();
}

double testForce::GetWakeFraction(double sidewashAngle) const {
  return FrPropellerForce::GetWakeFraction(sidewashAngle);
}

void testForce::ReadCoefficientsFile() {

}

GeneralizedForce testForce::ComputeGeneralizedForceInBody() {
  return GeneralizedForce();
}

testForce::testForce(const std::string& name, FrBody *body, Position propellerPositionInBody) :
    FrPropellerForce(name, body, propellerPositionInBody, NWU) {

}


class TestFrPropellerForce : public testing::Test {

 public:
  TestFrPropellerForce() : system("test_FrPropellerForce") {}

 protected:

  /// Initialization of the environment
  void SetUp() override;

 public:

  FRAME_CONVENTION fc = NWU;

  std::string m_coeffFilePath = "rdx022/propeller_BCP_1320f.json";

  FrOffshoreSystem system;
  std::shared_ptr<FrBody> body;

  std::shared_ptr<testForce> force;

};

void TestFrPropellerForce::SetUp() {

  body = system.NewBody("body");

//  auto propCoeffFile = FrFileSystem::join({system.config_file().GetDataFolder(), m_coeffFilePath});

  force = std::make_shared<testForce>("propulsionForce", body.get(), Position(-10, 0, 0));

  system.Initialize();
}

TEST_F(TestFrPropellerForce, GetWakeFraction) {

  ASSERT_FLOAT_EQ(force->GetWakeFraction(0.), 0.);
  force->SetStraightRunWakeFraction(0.4);
  ASSERT_FLOAT_EQ(force->GetWakeFraction(0.), 0.4);

}

TEST_F(TestFrPropellerForce, ComputeLongitudinalVelocity) {
  force->ComputeLongitudinalVelocity();

  //TODO
}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
