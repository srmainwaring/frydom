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

class TestFrStandardWindForce : public ::testing::Test {

 protected:

  FrOffshoreSystem system;
  std::shared_ptr<FrBody> body;
  std::shared_ptr<FrWindStandardForce> force;

  double m_frontalArea;
  double m_lateralArea;
  double m_lengthBetweenPerpendicular;
  double m_lengthOverAll;
  double m_Xcenter;
  double m_windSpeed;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_direction;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_dragCoefficient;

  double m_xadim;
  double m_yadim;
  double m_nadim;

  TestFrStandardWindForce() : system("TestFrStandardWindForce") {}

  void SetUp() override;

  void LoadData(std::string filename);


};

void TestFrStandardWindForce::LoadData(std::string filename) {

  std::string group = "/standardDNV/wind/";

  HighFive::File file(filename, HighFive::File::ReadOnly);
  m_frontalArea = H5Easy::load<double>(file, group + "FrontalArea");
  m_lateralArea = H5Easy::load<double>(file, group + "LateralArea");
  m_lengthBetweenPerpendicular = H5Easy::load<double>(file, group + "LengthBetweenPerpendicular");
  m_lengthOverAll = H5Easy::load<double>(file, group + "LengthOverAll");
  m_Xcenter = H5Easy::load<double>(file, group + "Xcenter");
  m_windSpeed = H5Easy::load<double>(file, group + "WindSpeed");
  m_direction = H5Easy::load<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(file, group + "WindDirection");
  m_dragCoefficient = H5Easy::load<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(file, group + "DragCoefficient");

}

void TestFrStandardWindForce::SetUp() {
  auto database = FrFileSystem::join({system.config_file().GetDataFolder(), "unit_test/TNR_database.h5"});
  LoadData(database);

  body = system.NewBody("body");

  force = make_wind_standard_force("wind_standard", body);
  force->SetLenghtBetweenPerpendicular(m_lengthBetweenPerpendicular);
  force->SetLateralArea(m_lateralArea);
  force->SetTransverseArea(m_frontalArea);
  force->SetXCenter(m_Xcenter);

  FrInertiaTensor InertiaTensor(1., 1., 1., 1., 0., 0., 0., Position(), NWU);
  body->SetInertiaTensor(InertiaTensor);

  system.GetEnvironment()->GetAtmosphere()->GetWind()->MakeFieldUniform();

  auto rho = system.GetEnvironment()->GetAtmosphere()->GetDensity();
  m_xadim = 0.5 * rho * m_windSpeed * m_windSpeed * m_frontalArea;
  m_yadim = 0.5 * rho * m_windSpeed * m_windSpeed * m_lateralArea;
  m_nadim = 0.5 * rho * m_windSpeed * m_windSpeed * m_lateralArea * m_lengthOverAll;

  system.Initialize();
}

TEST_F(TestFrStandardWindForce, TestForce) {

  for (int i = 0; i < m_direction.size(); i++) {

    system.GetEnvironment()->GetAtmosphere()->GetWind()->GetFieldUniform()
        ->Set(m_direction(i), m_windSpeed, DEG, MS, NED, COMEFROM);

    force->Update(0.);

    EXPECT_NEAR(m_dragCoefficient(0, i), force->GetForceInWorld(NWU).GetFx() / m_xadim, 1.e-5);
    EXPECT_NEAR(m_dragCoefficient(1, i), force->GetForceInWorld(NWU).GetFy() / m_yadim, 1.e-5);
    EXPECT_NEAR(m_dragCoefficient(2, i), force->GetTorqueInWorldAtCOG(NWU).GetMz() / m_nadim, 1.e-5);

  }
}

TEST_F(TestFrStandardWindForce, TestTransport) {

  int i = 2;
  double xc = 0.5;

  system.GetEnvironment()->GetAtmosphere()->GetWind()->GetFieldUniform()
      ->Set(m_direction(i), m_windSpeed, DEG, MS, NED, COMEFROM);

  FrInertiaTensor InertiaTensor(1., 1., 1., 1., 0., 0., 0., Position(0.1, 0., 0.), NWU);
  body->SetInertiaTensor(InertiaTensor);

  body->Initialize();

  force->SetXCenter(xc);

  force->Initialize();
  force->Update(0.);

  double torqueRef = m_dragCoefficient(2, i) * m_nadim + (xc - 0.1) * m_dragCoefficient(1, i) * m_yadim;
  EXPECT_NEAR(torqueRef / m_nadim, force->GetTorqueInWorldAtCOG(NWU).GetMz() / m_nadim, 1.e-5);
}
