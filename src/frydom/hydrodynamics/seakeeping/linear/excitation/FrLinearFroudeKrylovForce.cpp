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

#include "FrLinearFroudeKrylovForce.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOceanInc.h"
#include "frydom/logging/FrTypeNames.h"

namespace frydom {

  Eigen::MatrixXcd FrLinearFroudeKrylovForce::GetHDBData(unsigned int iangle) const {

    // Getter for the Froude-Krylov loads for a given wave direction.

    auto BEMBody = m_HDB->GetBody(GetBody());

    return BEMBody->GetFroudeKrylov(iangle);

  }

  Eigen::MatrixXcd FrLinearFroudeKrylovForce::GetHDBDataXDerivative(unsigned int iangle) const {

    // Getter for the x-derivative of the Froude-Krylov loads for a given wave direction.

    auto BEMBody = m_HDB->GetBody(GetBody());

    return BEMBody->GetXDerivativeFroudeKrylov(iangle);

  }

  Eigen::VectorXcd FrLinearFroudeKrylovForce::GetHDBData(unsigned int iangle, unsigned int iforce) const {

    // Getter for the Froude-Krylov loads for a given wave direction and dof.

    auto BEMBody = m_HDB->GetBody(GetBody());

    return BEMBody->GetFroudeKrylov(iangle, iforce);

  }

  Eigen::VectorXcd FrLinearFroudeKrylovForce::GetHDBDataXDerivative(unsigned int iangle, unsigned int iforce) const {

    // Getter for the x-derivative of the Froude-Krylov loads for a given wave direction and dof.

    auto BEMBody = m_HDB->GetBody(GetBody());

    return BEMBody->GetXDerivativeFroudeKrylov(iangle, iforce);

  }

  FrLinearFroudeKrylovForce::FrLinearFroudeKrylovForce(const std::string &name,
                                                       FrBody *body,
                                                       const std::shared_ptr<FrHydroDB> &HDB) :
      FrLinearHDBForce(name, TypeToString(this), body, HDB) {}

  std::shared_ptr<FrLinearFroudeKrylovForce>
  make_linear_froude_krylov_force(const std::string &name,
                                  std::shared_ptr<FrBody> body,
                                  std::shared_ptr<FrHydroDB> HDB) {

    // This function creates the linear Froude-Krylov force object.

    // Construction of the excitation force object from the HDB.
    auto LinFKForce = std::make_shared<FrLinearFroudeKrylovForce>(name, body.get(), HDB);

    // Add the excitation force object as an external force to the body.
    body->AddExternalForce(LinFKForce); // Initialization of m_body.

    return LinFKForce;

  }

}  // end namespace frydom
