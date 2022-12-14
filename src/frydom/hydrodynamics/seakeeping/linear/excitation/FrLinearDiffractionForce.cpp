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

#include "FrLinearDiffractionForce.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOceanInc.h"
#include "frydom/logging/FrTypeNames.h"

namespace frydom {

  Eigen::MatrixXcd FrLinearDiffractionForce::GetHDBData(unsigned int iangle) const {

    // Getter for the diffraction loads for a given wave direction.

    auto BEMBody = m_HDB->GetBody(GetBody());

    return BEMBody->GetDiffraction(iangle);

  }

  Eigen::MatrixXcd FrLinearDiffractionForce::GetHDBDataXDerivative(unsigned int iangle) const {

    // Getter for the x-derivative of the diffraction loads for a given wave direction.

    auto BEMBody = m_HDB->GetBody(GetBody());

    return BEMBody->GetXDerivativeDiffraction(iangle);

  }

  Eigen::VectorXcd FrLinearDiffractionForce::GetHDBData(unsigned int iangle, unsigned int iforce) const {

    // Getter for the diffraction loads for a given wave direction and dof.

    auto BEMBody = m_HDB->GetBody(GetBody());

    return BEMBody->GetDiffraction(iangle, iforce);

  }

  Eigen::VectorXcd FrLinearDiffractionForce::GetHDBDataXDerivative(unsigned int iangle, unsigned int iforce) const {

    /// Getter for the x-derivative of the diffraction loads for a given wave direction and dof.

    auto BEMBody = m_HDB->GetBody(GetBody());

    return BEMBody->GetXDerivativeDiffraction(iangle, iforce);

  }

  FrLinearDiffractionForce::FrLinearDiffractionForce(const std::string &name,
                                                     FrBody *body,
                                                     const std::shared_ptr<FrHydroDB> &HDB)
      : FrLinearHDBForce(name, TypeToString(this), body, HDB) {}

  std::shared_ptr<FrLinearDiffractionForce>
  make_linear_diffraction_force(const std::string &name,
                                std::shared_ptr<FrBody> body,
                                std::shared_ptr<FrHydroDB> HDB) {

    // Construction of the excitation force object from the HDB.
    auto diffractionForce = std::make_shared<FrLinearDiffractionForce>(name, body.get(), HDB);

    // Add the excitation force object as an external force to the body.
    body->AddExternalForce(diffractionForce);

    return diffractionForce;

  }

}  // end namespace frydom
