//
// Created by frongere on 07/05/2020.
//

#include "Fr3DGeometry.h"


namespace frydom {

  FrCylinder::FrCylinder(const double &radius, const double &height) :
      m_radius(radius),
      m_height(height) {}

  void FrCylinder::SetRadius(const double &radius) {
    m_radius = radius;
  }

  double FrCylinder::GetRadius() const {
    return m_radius;
  }

  void FrCylinder::SetHeight(const double &height) {
    m_height = height;
  }

  double FrCylinder::GetHeight() const {
    return m_height;
  }

  double FrCylinder::GetVolume() const {
    return m_height * MU_PI * m_radius * m_radius;
  }

  FrInertiaTensor FrCylinder::GetUnitInertiaTensor() const {
    double Ixx = m_radius * m_radius / 4. + m_height * m_height / 12.;
    double Iyy = Ixx;
    double Izz = m_radius * m_radius / 2.;

    return FrInertiaTensor(1., Ixx, Iyy, Izz, 0., 0., 0., Position(), NWU);
  }


}  // end namespace frydom
