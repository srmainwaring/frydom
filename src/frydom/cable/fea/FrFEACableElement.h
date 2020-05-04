//
// Created by frongere on 29/04/2020.
//

#ifndef FRYDOM_FRFEACABLEELEMENT_H
#define FRYDOM_FRFEACABLEELEMENT_H

#include "chrono/fea/ChElementBeamIGA.h"

#include "FrFEANode.h"


namespace frydom {

  namespace internal {

    class FrFEACableElementBase : public chrono::fea::ChElementBeamIGA {

     public:

      /// Gets the absolute xyz velocity of a point on the beam line, at abscissa 'eta'.
      /// Note, eta=-1 at node1, eta=+1 at node2.
      virtual void EvaluateSectionSpeed(const double eta,
                                        chrono::ChVector<> &point_speed);

      /// Gets the absolute xyz position of a point on the beam line, at abscissa 'eta'.
      /// Note, eta=-1 at node1, eta=+1 at node2.
      virtual void EvaluateSectionAcceleration(const double eta,
                                               chrono::ChVector<> &point_acceleration);

    };

  }  // end namespace frydom::internal


}  // end namespace frydom



#endif //FRYDOM_FRFEACABLEELEMENT_H
