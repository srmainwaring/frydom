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

class TestITTCResistance : public ::testing::Test {

 protected:

  double LengthBetweenPerpendicular;
  double hullFormFactor;
  double hullWetSurface;
  double frontalArea;
  double m_cr;
  double m_ca;
  double m_ct;
  double m_cf;
  double m_caa;
  double m_capp;
  double speed;
  double reynoldsNumber;
  double lengthAtWaterLine;
  double m_waterDensity;

  FrOffshoreSystem system;
  std::shared_ptr<FrBody> body;
  std::shared_ptr<FrITTCResistance> force;

  TestITTCResistance() : system("test_FrITTCResistance") {}

  void SetUp() override;

  void LoadData(std::string filename);

};

void TestITTCResistance::SetUp() {
  body = system.NewBody("body");
  body->SetPosition(Position(105., -2., 0.3), NWU);
  auto direction = Direction(0.1, 0.3, 0.9);
  direction.normalize();
  body->SetRotation(FrUnitQuaternion(direction, M_PI / 5., NWU));

  FrInertiaTensor InertiaTensor(1., 1., 1., 1., 0., 0., 0., Position(0.2, 0.2, 0.1), NWU);
  body->SetInertiaTensor(InertiaTensor);

  system.GetEnvironment()->GetOcean()->GetCurrent()->MakeFieldUniform();
  auto database = FrFileSystem::join({system.config_file().GetDataFolder(), "unit_test/TNR_database.h5"});
  LoadData(database);
}

void TestITTCResistance::LoadData(std::string filename) {
  
  std::string group = "/ittc/";
  
  HighFive::File file(filename, HighFive::File::ReadOnly);

  LengthBetweenPerpendicular = H5Easy::load<double>(file, group + "LengthBetweenPerpendicular");
  hullFormFactor = H5Easy::load<double>(file, group + "HullFormParameter");
  hullWetSurface = H5Easy::load<double>(file, group + "WettedSurfaceArea");
  frontalArea = H5Easy::load<double>(file, group + "FrontalArea");
  lengthAtWaterLine = H5Easy::load<double>(file, group + "LengthAtTheWaterLine");
  reynoldsNumber = H5Easy::load<double>(file, group + "ReynoldsNumber");

  speed = H5Easy::load<double>(file, group + "Speed");

  m_ct = H5Easy::load<double>(file, group + "CT");
  m_cr = H5Easy::load<double>(file, group + "CR");
  m_cf = H5Easy::load<double>(file, group + "CF");
  m_ca = H5Easy::load<double>(file, group + "CA");
  m_caa = H5Easy::load<double>(file, group + "CAA");
  m_capp = H5Easy::load<double>(file, group + "CAPP");

  m_waterDensity = H5Easy::load<double>(file, group + "waterDensity");
  system.GetEnvironment()->GetOcean()->SetDensity(m_waterDensity);
  auto kinematicViscosity = H5Easy::load<double>(file, group + "KinematicViscosity");
  system.GetEnvironment()->GetOcean()->SetKinematicViscosity(kinematicViscosity);
}

TEST_F(TestITTCResistance, test0) {
  force = make_ITTC_resistance_force("test_FrITTCResistance", body, LengthBetweenPerpendicular,
                                     hullWetSurface, m_cr);
  force->SetHullFormFactor(hullFormFactor);
  force->SetRoughnessFromLength(lengthAtWaterLine);
  force->SetAirResistanceFromArea(frontalArea);
  force->SetAppendageCoefficient(m_capp);

  system.Initialize();

  body->SetVelocityInBodyNoRotation(Velocity(speed, 0, 0), NWU);
  force->Update(0.);

  auto ct = force->GetForceInBody(NWU) / (-0.5 * m_waterDensity * speed * speed * hullWetSurface);

  EXPECT_NEAR(m_ct, ct.x(), 1e-8);
  EXPECT_NEAR(0., ct.y(), 1e-8);
  EXPECT_NEAR(0., ct.z(), 1e-8);
}
