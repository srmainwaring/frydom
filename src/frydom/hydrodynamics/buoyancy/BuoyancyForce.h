//
// Created by camille on 25/02/2020.
//

#ifndef FRYDOM_BUOYANCYFORCE_H
#define FRYDOM_BUOYANCYFORCE_H

#include "frydom/frydom.h"

namespace frydom {

  class BuoyancyForce : public FrForce {

   public:

    BuoyancyForce(const std::string& name, FrBody* body, double volume, Position pos);

    bool IncludedInStaticAnalysis() const override { return true; }

   protected:

    void Compute(double time) override;

   private:

    Position m_pos;
    double m_volume;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

} // end namespace frydom

#endif //FRYDOM_BUOYANCYFORCE_H
