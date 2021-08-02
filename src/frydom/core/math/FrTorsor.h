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

#ifndef FRYDOM_FRTORSOR_H
#define FRYDOM_FRTORSOR_H

#include "FrVector.h"

namespace frydom {

  class FrTorsor {

   public:

    FrTorsor(const mathutils::Vector3d<double> &resultant, const mathutils::Vector3d<double> &moment,
             const Position &point, FRAME_CONVENTION fc);

    Position GetPoint(FRAME_CONVENTION fc) const;


   protected:

    void
    Set(const mathutils::Vector3d<double> &resultant, const mathutils::Vector3d<double> &moment, const Position &point,
        FRAME_CONVENTION fc);

    mathutils::Vector3d<double> m_resultant;  ///< resultant of the torsor
    mathutils::Vector3d<double> m_moment;     ///< moment of the torsor, expressed at the point
    Position m_point;                         ///< Application point, where the torsor is expressed

    mathutils::Vector3d<double> TransportMomentAtPoint(const Position &newPoint, FRAME_CONVENTION fc) const;

//    FrFrame m_frame;

   private:

    friend std::ostream &operator<<(std::ostream &os, const FrTorsor &torsor);

    std::ostream &cout(std::ostream &os) const;
  };


  class GeneralizedForceTorsor : public FrTorsor {

   public:

    GeneralizedForceTorsor(const Force &force, const Torque &torque, const Position &point, FRAME_CONVENTION fc);

    GeneralizedForceTorsor(const GeneralizedForce generalizedForce, const Position &point, FRAME_CONVENTION fc);

    void Set(const Force &force, const Torque &torque, const Position &point, FRAME_CONVENTION fc);

    Force GetForce() const;

    Torque GetTorqueAtPoint(const Position &point, FRAME_CONVENTION fc) const;

  };


  class GeneralizedVelocityTorsor : public FrTorsor {

   public:

    GeneralizedVelocityTorsor(const Velocity &linearvelocity, const AngularVelocity &angularVelocity,
                              const Position &point, FRAME_CONVENTION fc);

    void Set(const Velocity &linearvelocity, const AngularVelocity &angularVelocity, const Position &point,
             FRAME_CONVENTION fc);

    Force GetAngularVelocity() const;

    Torque GetLinearVelocityAtPoint(const Position &point, FRAME_CONVENTION fc) const;

  };

} //end namespace frydom

#endif //FRYDOM_FRTORSOR_H
