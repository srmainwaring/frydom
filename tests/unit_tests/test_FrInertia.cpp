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

#include <highfive/H5File.hpp>
#include <highfive/H5Easy.hpp>

using namespace frydom;


class TestInertia : public ::testing::Test {

 protected:

  FrOffshoreSystem system;
  std::shared_ptr<FrBody> body;
  std::shared_ptr<FrInertiaTensor> inertia;

  Position m_BodyPositionInWorld;
  Direction m_BodyRotationDirection;
  double m_BodyRotationAngle;
  Position m_COG;
  double m_BodyMass;

  Position m_PointInBody;
  Direction m_FrameRotationDirection;
  double m_FrameRotationAngle;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_InertialInFrameAtPoint;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_InertialInBodyAtCOG;


  TestInertia() : system("test_FrInertia") {}

  void SetUp() override;

  void LoadData(std::string filename);

  void CheckInertiaAtCOG() const;

  void CheckInertiaAtPoint() const;

};

void TestInertia::LoadData(std::string filename) {

  HighFive::File file(filename, HighFive::File::ReadOnly);
  std::string group = "/inertia/";

  m_BodyPositionInWorld = H5Easy::load<Eigen::Matrix<double, 3, 1>>(file, group + "BodyPositionInWorld");
  m_BodyRotationDirection = H5Easy::load<Eigen::Matrix<double, 3, 1>>(file, group + "BodyRotationDirection");
  m_BodyRotationAngle = H5Easy::load<double>(file, group + "BodyRotationAngle");
  m_COG = H5Easy::load<Eigen::Matrix<double, 3, 1>>(file, group + "COG");
  m_PointInBody = H5Easy::load<Eigen::Matrix<double, 3, 1>>(file, group + "PointInBody");
  m_FrameRotationDirection = H5Easy::load<Eigen::Matrix<double, 3, 1>>(file, group + "FrameRotationDirection");
  m_FrameRotationAngle = H5Easy::load<double>(file, group + "FrameRotationAngle");
  m_BodyMass = H5Easy::load<double>(file, group + "BodyMass");
  m_InertialInBodyAtCOG = H5Easy::load<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(file, group + "InertiaInBodyAtCOG");
  m_InertialInFrameAtPoint = H5Easy::load<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(file, group + "InertiaInFrameAtPoint");

}

void TestInertia::SetUp() {

  auto database = FrFileSystem::join({system.config_file().GetDataFolder(), "unit_test/TNR_database.h5"});
  this->LoadData(database);

  body = system.NewBody("body");
  body->SetPosition(m_BodyPositionInWorld, NWU);
  body->SetRotation(FrUnitQuaternion(m_BodyRotationDirection, m_BodyRotationAngle, NWU));

  body->SetInertiaTensor(FrInertiaTensor(m_BodyMass, 1., 1., 1., 0., 0., 0., m_COG, NWU));
//    body->SetInertiaTensor(FrInertiaTensor(m_BodyMass,m_COG,NWU));
//    body->SetCOG(m_COG, NWU);
//    body->SetMass(m_BodyMass);

  system.Initialize();

  body->GetFrameAtCOG(NWU);

}

void TestInertia::CheckInertiaAtCOG() const {
  double Ixx, Iyy, Izz;
  double Ixy, Ixz, Iyz;
  inertia->GetInertiaCoeffsAtCOG(Ixx, Iyy, Izz, Ixy, Ixz, Iyz, NWU);

  EXPECT_NEAR(m_InertialInBodyAtCOG(0, 0), Ixx, 1e-8);
  EXPECT_NEAR(m_InertialInBodyAtCOG(1, 1), Iyy, 1e-8);
  EXPECT_NEAR(m_InertialInBodyAtCOG(2, 2), Izz, 1e-8);
  EXPECT_NEAR(m_InertialInBodyAtCOG(0, 1), Ixy, 1e-8);
  EXPECT_NEAR(m_InertialInBodyAtCOG(0, 2), Ixz, 1e-8);
  EXPECT_NEAR(m_InertialInBodyAtCOG(1, 2), Iyz, 1e-8);
}


void TestInertia::CheckInertiaAtPoint() const {

  auto frame = FrFrame(m_PointInBody, FrRotation(m_FrameRotationDirection, m_FrameRotationAngle, NWU), NWU);

  double Ixx, Iyy, Izz;
  double Ixy, Ixz, Iyz;
  inertia->GetInertiaCoeffsAtFrame(Ixx, Iyy, Izz, Ixy, Ixz, Iyz, frame, NWU);

  EXPECT_NEAR(m_InertialInFrameAtPoint(0, 0), Ixx, 1e-8);
  EXPECT_NEAR(m_InertialInFrameAtPoint(1, 1), Iyy, 1e-8);
  EXPECT_NEAR(m_InertialInFrameAtPoint(2, 2), Izz, 1e-8);
  EXPECT_NEAR(m_InertialInFrameAtPoint(0, 1), Ixy, 1e-8);
  EXPECT_NEAR(m_InertialInFrameAtPoint(0, 2), Ixz, 1e-8);
  EXPECT_NEAR(m_InertialInFrameAtPoint(1, 2), Iyz, 1e-8);

}

TEST_F(TestInertia, InertiaInFrame) {

  auto frame = FrFrame(m_PointInBody, FrRotation(m_FrameRotationDirection, m_FrameRotationAngle, NWU), NWU);

  inertia = std::make_shared<FrInertiaTensor>(m_BodyMass,
                                              m_InertialInFrameAtPoint(0, 0), m_InertialInFrameAtPoint(1, 1),
                                              m_InertialInFrameAtPoint(2, 2),
                                              m_InertialInFrameAtPoint(0, 1), m_InertialInFrameAtPoint(0, 2),
                                              m_InertialInFrameAtPoint(1, 2),
                                              frame, body->GetCOG(NWU), NWU);

  this->CheckInertiaAtCOG();

  this->CheckInertiaAtPoint();
}

TEST_F(TestInertia, InertiaAtCOG) {

  inertia = std::make_shared<FrInertiaTensor>(m_BodyMass,
                                              m_InertialInBodyAtCOG(0, 0), m_InertialInBodyAtCOG(1, 1),
                                              m_InertialInBodyAtCOG(2, 2),
                                              m_InertialInBodyAtCOG(0, 1), m_InertialInBodyAtCOG(0, 2),
                                              m_InertialInBodyAtCOG(1, 2),
                                              body->GetCOG(NWU), NWU);

  this->CheckInertiaAtCOG();
}

TEST_F(TestInertia, BodyInertiaAtCOG) {

  FrInertiaTensor InertiaTensor(m_BodyMass,
                                m_InertialInBodyAtCOG(0, 0), m_InertialInBodyAtCOG(1, 1), m_InertialInBodyAtCOG(2, 2),
                                m_InertialInBodyAtCOG(0, 1), m_InertialInBodyAtCOG(0, 2), m_InertialInBodyAtCOG(1, 2),
                                body->GetCOG(NWU), NWU);
  body->SetInertiaTensor(InertiaTensor);

  inertia = std::make_shared<FrInertiaTensor>(body->GetInertiaTensor());
  this->CheckInertiaAtCOG();
}

TEST_F(TestInertia, BodyInertiaInFrame) {

  auto frame = FrFrame(m_PointInBody, FrRotation(m_FrameRotationDirection, m_FrameRotationAngle, NWU), NWU);


  body->SetInertiaTensor(FrInertiaTensor(m_BodyMass,
                                         m_InertialInFrameAtPoint(0, 0), m_InertialInFrameAtPoint(1, 1),
                                         m_InertialInFrameAtPoint(2, 2),
                                         m_InertialInFrameAtPoint(0, 1), m_InertialInFrameAtPoint(0, 2),
                                         m_InertialInFrameAtPoint(1, 2),
                                         frame, body->GetCOG(NWU), NWU));

  inertia = std::make_shared<FrInertiaTensor>(body->GetInertiaTensor());
  this->CheckInertiaAtCOG();

  this->CheckInertiaAtPoint();
}

TEST_F(TestInertia, BodyInertia) {

  body->SetInertiaTensor(FrInertiaTensor(m_BodyMass,
                                         m_InertialInBodyAtCOG(0, 0), m_InertialInBodyAtCOG(1, 1),
                                         m_InertialInBodyAtCOG(2, 2),
                                         m_InertialInBodyAtCOG(0, 1), m_InertialInBodyAtCOG(0, 2),
                                         m_InertialInBodyAtCOG(1, 2),
                                         body->GetCOG(NWU), NWU));

  inertia = std::make_shared<FrInertiaTensor>(body->GetInertiaTensor());
  this->CheckInertiaAtCOG();
}

