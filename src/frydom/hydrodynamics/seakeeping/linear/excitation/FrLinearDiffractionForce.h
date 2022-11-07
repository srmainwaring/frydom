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


#ifndef FRYDOM_FRLINEARDIFFRACTIONFORCE_H
#define FRYDOM_FRLINEARDIFFRACTIONFORCE_H

#include <memory>
#include <vector>

#include "MathUtils/Matrix66.h"
#include "frydom/core/force/FrForce.h"
#include "FrLinearHDBForce.h"

#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"

namespace frydom {

  // Forward declaration
  class FrHydroDB;

  class FrBody;

  class FrEquilibriumFrame;

  /**
   * \class FrLinearDiffractionForce
   * \brief Class for computing the linear diffraction loads.
   */
  class FrLinearDiffractionForce : public FrLinearHDBForce {

   public:

    /// Constructor.
    explicit FrLinearDiffractionForce(const std::string &name,
                                      FrBody *body,
                                      const std::shared_ptr<FrHydroDB> &HDB);

    /// Getter for the diffraction loads for a given wave direction.
    Eigen::MatrixXcd GetHDBData(unsigned int iangle) const override;

    /// Getter for the x-derivative of the diffraction loads for a given wave direction.
    Eigen::MatrixXcd GetHDBDataXDerivative(unsigned int iangle) const override;

    /// Getter for the diffraction loads for a given wave direction and dof.
    Eigen::VectorXcd GetHDBData(unsigned int iangle, unsigned int iforce) const override;

    /// Getter for the x-derivative of the diffraction loads for a given wave direction and dof.
    Eigen::VectorXcd GetHDBDataXDerivative(unsigned int iangle, unsigned int iforce) const override;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  std::shared_ptr<FrLinearDiffractionForce>
  make_linear_diffraction_force(const std::string &name,
                                std::shared_ptr<FrBody> body,
                                std::shared_ptr<FrHydroDB> HDB);


}  // end namespace frydom

#endif //FRYDOM_FRLINEARDIFFRACTIONFORCE_H
