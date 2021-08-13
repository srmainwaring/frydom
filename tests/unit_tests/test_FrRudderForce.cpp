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

class TestFrRudderForce : public testing::Test {

 public:
  TestFrRudderForce() : system("test_FrRudderForce") {}

 protected:

  /// Initialization of the environment
  void SetUp() override;

 public:

  FRAME_CONVENTION fc = NWU;

  std::string m_coeffFilePath = "rdx022/flap_rudder.json";

  FrOffshoreSystem system;
  std::shared_ptr<FrBody> body;

  std::shared_ptr<FrRudderForce> force;

};

void TestFrRudderForce::SetUp() {

  body = system.NewBody("body");
//  body->SetPosition(m_PositionInWorld, fc);
//  body->SetRotation(m_quat);

  auto node = body->NewNode("rudderNode");
  node->SetPositionInBody(Position(-10., 0., 0.), NWU);

  auto rudderCoeffFile = FrFileSystem::join({system.config_file().GetDataFolder(), m_coeffFilePath});

  force = make_rudder_force("rudderForce", body, node, rudderCoeffFile);

  system.Initialize();
}

//TEST_F(TestFrRudderForce, ReadCoefficientsFile) {
//  TODO
//}
//
//TEST_F(TestFrRudderForce, ComputeGeneralizedForceInWorld) {
//  TODO
//}

TEST_F(TestFrRudderForce, GetDriftAngle) {

  ASSERT_FLOAT_EQ(force->GetDriftAngle(Velocity(1., 0., 0.)), 0.);
  ASSERT_FLOAT_EQ(force->GetDriftAngle(Velocity(0., 1., 0.)), MU_PI_2);
  ASSERT_FLOAT_EQ(force->GetDriftAngle(Velocity(0., 0., 1.)), 0.);
  Velocity randVel; randVel.setRandom();
  ASSERT_FLOAT_EQ(force->GetDriftAngle(randVel), atan2(randVel.GetVy(), randVel.GetVx()));

  body->Rotate(FrRotation(UP(NWU), MU_PI_2, NWU));
  ASSERT_FLOAT_EQ(force->GetDriftAngle(randVel), atan2(-randVel.GetVx(), randVel.GetVy()));

}

TEST_F(TestFrRudderForce, GetWakeFraction) {

  ASSERT_FLOAT_EQ(force->GetWakeFraction(0.), 0.4);

  double sidewashAngle = MU_PI_4;
  ASSERT_FLOAT_EQ(force->GetWakeFraction(sidewashAngle), 0.4*exp(-4*sidewashAngle*sidewashAngle));

  auto wp0 = 0.8;
  force->SetStraightRunWakeFraction(wp0);
  ASSERT_FLOAT_EQ(force->GetWakeFraction(0.), wp0);
  ASSERT_FLOAT_EQ(force->GetWakeFraction(sidewashAngle), wp0*exp(-4*sidewashAngle*sidewashAngle));

}

TEST_F(TestFrRudderForce, ComputeSpecialSidewashAngle) {

  // Init, body without velocity
  ASSERT_FLOAT_EQ(force->ComputeSpecialSidewashAngle(), 0.);

  // body with a generalized velocity
  AngularVelocity bodyAngVel(0., 0., 1.);
  Velocity bodyVel(1., 0., 0.);
  body->SetGeneralizedVelocityInWorld(bodyVel, bodyAngVel, NWU);
  ASSERT_FLOAT_EQ(force->ComputeSpecialSidewashAngle(), atan2(-20, 1));

  // modification of the transverse velocity correction
  force->SetTransverseVelocityCorrection(3.);
  ASSERT_FLOAT_EQ(force->ComputeSpecialSidewashAngle(), atan2(-30, 1));

};

TEST_F(TestFrRudderForce, Kappa) {

//  |beta| < beta_1
  ASSERT_FLOAT_EQ(force->Kappa(0.), 0.);

  ASSERT_FLOAT_EQ(force->Kappa(0.5), 0.45*0.5);

  ASSERT_FLOAT_EQ(force->Kappa(-0.5), 0.45*0.5);

  ASSERT_FLOAT_EQ(force->Kappa(1.111112), 0.5);

  ASSERT_FLOAT_EQ(force->Kappa(1.2999), 0.5);

//  beta_1 <= |beta| <= beta_2
  auto fct = [](double beta) { return 0.5 * (1 - 1.3/(MU_PI_2-1.3) + beta/(MU_PI_2-1.3)); };
  ASSERT_FLOAT_EQ(force->Kappa(1.3), fct(1.3));
  ASSERT_FLOAT_EQ(force->Kappa(MU_PI_2), fct(MU_PI_2));

//  |beta| > beta_2
  ASSERT_FLOAT_EQ(force->Kappa(MU_PI), 1.);

}

TEST_F(TestFrRudderForce, GetInflowVelocityInWorld) {

  // Init, without hull/rudder interaction and body/current velocity
  ASSERT_FLOAT_EQ(force->GetRudderRelativeVelocityInWorld().norm(), 0.);

  // with current velocity
  system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
  auto currentField = system.GetEnvironment()->GetOcean()->GetCurrent()->GetFieldUniform();

  Velocity currentVel = 2.*NORTH(NWU);
  currentField->Set(currentVel, NWU, COMEFROM);

  ASSERT_TRUE(force->GetRudderRelativeVelocityInWorld() == -currentVel);

  // with current and body linear velocity
  body->SetVelocityInWorldNoRotation(-currentVel, NWU);
  ASSERT_FLOAT_EQ(force->GetRudderRelativeVelocityInWorld().norm(), 0.);

  // with body generalized velocity
  AngularVelocity bodyAngVel(0., 0., 1.);
  body->SetGeneralizedVelocityInWorld(Velocity(), bodyAngVel, NWU);
  Velocity rudderRelativeVelocity = -(currentVel + bodyAngVel.cross(Position(-10, 0, 0)));
  ASSERT_TRUE(force->GetRudderRelativeVelocityInWorld() == rudderRelativeVelocity);

  // With hull/rudder interaction
  force->ActivateHullRudderInteraction(true);
  auto rudderRelativeVelocityInBody = body->ProjectVectorInBody(rudderRelativeVelocity, NWU);
  auto sidewashAngle = rudderRelativeVelocityInBody.GetProjectedAngleAroundZ(RAD);
  auto uRA = rudderRelativeVelocityInBody.GetVx() * (1. - force->GetWakeFraction(sidewashAngle));
  auto specialSidewashAngle = force->ComputeSpecialSidewashAngle();
  auto vRA = rudderRelativeVelocityInBody.GetVy() * force->Kappa(specialSidewashAngle);
  ASSERT_TRUE(force->GetRudderRelativeVelocityInWorld() == body->ProjectVectorInWorld(Velocity(uRA, vRA, 0.0), NWU));

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
