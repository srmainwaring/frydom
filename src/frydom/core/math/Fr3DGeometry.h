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

#ifndef FRYDOM_FR3DGEOMETRY_H
#define FRYDOM_FR3DGEOMETRY_H

#include "frydom/core/math/FrVector.h"
#include "frydom/core/body/FrInertiaTensor.h"


namespace frydom {

  struct Fr3DGeometryBase {
    virtual ~Fr3DGeometryBase();

//    virtual Position GetRelativePositionForNode(FRAME_CONVENTION fc) const = 0;

    virtual double GetVolume() const = 0;

    virtual FrInertiaTensor GetUnitInertiaTensor() const = 0;

    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };


  struct FrCylinder : public Fr3DGeometryBase {

    FrCylinder(const double &radius, const double &height);

//    Position GetRelativePositionForNode(FRAME_CONVENTION fc) const override;

    double GetVolume() const override;

    FrInertiaTensor GetUnitInertiaTensor() const override;

    void SetRadius(const double &radius);

    double GetRadius() const;

    void SetHeight(const double &height);

    double GetHeight() const;


    double m_radius;
    double m_height;

    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

}  // end namespace frydom



#endif //FRYDOM_FR3DGEOMETRY_H
