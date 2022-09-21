//
// Created by camille on 10/04/2020.
//

#ifndef FRYDOM_FRVARIABLESADDEDMASS_H
#define FRYDOM_FRVARIABLESADDEDMASS_H

#include "frydom/core/common/FrVariablesBodyBase.h"

namespace frydom {

  namespace internal {

    class FrVariablesAddedMass : public FrVariablesBodyBase {

     public:

      explicit FrVariablesAddedMass(const mathutils::Matrix66<double> &added_mass, chrono::ChVariablesBodyOwnMass *variables);

      void SetAddedMass(const mathutils::Matrix66<double>& added_mass);

      mathutils::Matrix66<double> &GetAddedMass();

      const mathutils::Matrix66<double> &GetAddedMass() const;

     protected:

      mathutils::Matrix66<double> m_added_mass;
      mathutils::Matrix66<double> m_body_mass;

     public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };

  }// end namespace internal
} // end namespace frydom

#endif //FRYDOM_FRVARIABLESADDEDMASS_H
