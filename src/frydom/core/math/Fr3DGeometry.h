//
// Created by frongere on 07/05/2020.
//

#ifndef FRYDOM_FR3DGEOMETRY_H
#define FRYDOM_FR3DGEOMETRY_H

#include "frydom/core/math/FrVector.h"
#include "frydom/core/body/FrInertiaTensor.h"


namespace frydom {

  struct Fr3DGeometryBase {

//    virtual Position GetRelativePositionForNode(FRAME_CONVENTION fc) const = 0;

    virtual double GetVolume() const = 0;

    virtual FrInertiaTensor GetUnitInertiaTensor() const = 0;

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
  };

}  // end namespace frydom



#endif //FRYDOM_FR3DGEOMETRY_H
