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

#include "frydom/environment/flow/FrFlowBase.h"

#include <highfive/H5File.hpp>
#include <highfive/H5Easy.hpp>

using namespace frydom;

// --------------------------------------------------------------------
//
// TEST OF THE FLOW BASE
//
// --------------------------------------------------------------------

class TestFrFlowBase : public ::testing::Test {

 protected:
  FrOffshoreSystem system;               ///< system environment
  std::shared_ptr<FrFlowBase> flow;      ///< flow instantiation for the test
  Velocity m_VelocityInWorld;            ///< Uniform velocity of the flow in world
  Velocity m_RelativeVelocityInFrame;    ///< Velocity of the flow relative to the frame express into the frame
  Position m_PointInWorld;               ///< Position of the frame in world (NWU)
  Velocity m_FrameVelocityInWorld;       ///< Velocity of the frame in world (NWU)
  FrUnitQuaternion m_quat;               ///< Orientation of the frame (quaternion) / world
  FrFrame m_frame;                       ///< Local frame

 protected:

  TestFrFlowBase() : system("test_FrFlowBase") {}

  /// Initialization of the environment
  void SetUp() override;

  /// Load reference results from HDF5 file
  void LoadData(std::string filename);

 public:
  void TestGetFluxVelocityInWorld();

  void TestGetFluxVelocityInFrame();

  // NewField()
  // GetField()

};

void TestFrFlowBase::SetUp() {

  auto database = FrFileSystem::join({system.config_file().GetDataFolder(), "unit_test/TNR_database.h5"});
  LoadData(database);
  system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
  flow = std::make_shared<FrCurrent>(system.GetEnvironment()->GetOcean());
  flow->MakeFieldUniform();
  flow->GetFieldUniform()->Set(m_VelocityInWorld, NWU, GOTO);
}

void TestFrFlowBase::LoadData(std::string filename) {

  HighFive::File file(filename, HighFive::File::ReadOnly);
  std::string group = "/flow_base/uniform/";

  m_PointInWorld = H5Easy::load<Eigen::Matrix<double, 3, 1>>(file, group + "PointInWorld");
  auto direction = H5Easy::load<Eigen::Matrix<double, 3, 1>>(file, group + "RotationDirection");
  direction.normalize();
  auto angle = H5Easy::load<double>(file, group + "RotationAngle");
  m_quat = FrUnitQuaternion(direction, angle, NWU);
  m_frame = FrFrame(m_PointInWorld, m_quat, NWU);

  m_FrameVelocityInWorld    = H5Easy::load<Eigen::Matrix<double, 3, 1>>(file, group + "FrameVelocityInWorld");
  m_VelocityInWorld         = H5Easy::load<Eigen::Matrix<double, 3, 1>>(file, group + "VelocityInWorld");
  m_RelativeVelocityInFrame = H5Easy::load<Eigen::Matrix<double, 3, 1>>(file, group + "RelativeVelocityInFrame");

}

void TestFrFlowBase::TestGetFluxVelocityInWorld() {

  auto velocity = flow->GetFluxVelocityInWorld(m_PointInWorld, NWU);
  EXPECT_FLOAT_EQ(velocity.GetVx(), m_VelocityInWorld.GetVx());
  EXPECT_FLOAT_EQ(velocity.GetVy(), m_VelocityInWorld.GetVy());
  EXPECT_FLOAT_EQ(velocity.GetVz(), m_VelocityInWorld.GetVz());

  velocity = flow->GetFluxVelocityInWorld(m_PointInWorld, NED);
  EXPECT_FLOAT_EQ(velocity.GetVx(), m_VelocityInWorld.GetVx());
  EXPECT_FLOAT_EQ(velocity.GetVy(), -m_VelocityInWorld.GetVy());
  EXPECT_FLOAT_EQ(velocity.GetVz(), -m_VelocityInWorld.GetVz());

}

void TestFrFlowBase::TestGetFluxVelocityInFrame() {

  auto velocity = flow->GetFluxRelativeVelocityInFrame(m_frame, m_FrameVelocityInWorld, NWU);
  EXPECT_FLOAT_EQ(velocity.GetVx(), m_RelativeVelocityInFrame.GetVx());
  EXPECT_FLOAT_EQ(velocity.GetVy(), m_RelativeVelocityInFrame.GetVy());
  EXPECT_FLOAT_EQ(velocity.GetVz(), m_RelativeVelocityInFrame.GetVz());

  auto FrameVelocityInWorld_NED = Velocity(m_FrameVelocityInWorld.GetVx(),
                                           -m_FrameVelocityInWorld.GetVy(),
                                           -m_FrameVelocityInWorld.GetVz());
  velocity = flow->GetFluxRelativeVelocityInFrame(m_frame, FrameVelocityInWorld_NED, NED);
  EXPECT_FLOAT_EQ(velocity.GetVx(), m_RelativeVelocityInFrame.GetVx());
  EXPECT_FLOAT_EQ(velocity.GetVy(), m_RelativeVelocityInFrame.GetVy());
  EXPECT_FLOAT_EQ(velocity.GetVz(), m_RelativeVelocityInFrame.GetVz());
}


TEST_F(TestFrFlowBase, GetFluxVelocityInWorld) {
  TestGetFluxVelocityInWorld();
}

TEST_F(TestFrFlowBase, GetRelativeVelocityInFrame) {
  TestGetFluxVelocityInFrame();
}

TEST_F(TestFrFlowBase, Update) {
  flow->Update(0.);
}

TEST_F(TestFrFlowBase, Initialize) {
  flow->Initialize();
}

TEST_F(TestFrFlowBase, StepFinalize) {
  flow->StepFinalize();
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
