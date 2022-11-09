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

#include "Fr3DGeometry.h"


namespace frydom {
  Fr3DGeometryBase::~Fr3DGeometryBase() {
  }

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
