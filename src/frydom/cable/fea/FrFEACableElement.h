//
// Created by frongere on 29/04/2020.
//

#ifndef FRYDOM_FRFEACABLEELEMENT_H
#define FRYDOM_FRFEACABLEELEMENT_H

#include "chrono/fea/ChElementBeamIGA.h"

#include "FrFEANode.h"


namespace frydom {

  class FrEnvironment;

  namespace internal {

    class FrFEACableElementBase : public chrono::fea::ChElementBeamIGA {

     public:

      explicit FrFEACableElementBase(FrEnvironment* environment);

      /// Gets the absolute xyz velocity of a point on the beam line, at abscissa 'eta'.
      /// Note, eta=-1 at node1, eta=+1 at node2.
      virtual void EvaluateSectionSpeed(const double eta,
                                        chrono::ChVector<> &point_speed);

      /// Gets the absolute xyz position of a point on the beam line, at abscissa 'eta'.
      /// Note, eta=-1 at node1, eta=+1 at node2.
      virtual void EvaluateSectionAcceleration(const double eta,
                                               chrono::ChVector<> &point_acceleration);

      /// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
      /// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.
      void ComputeKRMmatricesGlobal(chrono::ChMatrix<> &H,
                                    double Kfactor,
                                    double Rfactor = 0,
                                    double Mfactor = 0) override;

//      void SetupInitial(chrono::ChSystem* system) override;

      chrono::ChVector<double> GetTangent(const double eta);

     public:
      FrEnvironment* m_environment;

    };

  }  // end namespace frydom::internal


}  // end namespace frydom



#endif //FRYDOM_FRFEACABLEELEMENT_H
