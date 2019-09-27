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

namespace frydom {

    Eigen::MatrixXcd FrLinearDiffractionForce::GetHDBData(unsigned int iangle) const {

      auto BEMBody = m_HDB->GetBody(m_body);

      return BEMBody->GetDiffraction(iangle);

    }

    Eigen::VectorXcd FrLinearDiffractionForce::GetHDBData(unsigned int iangle, unsigned int iforce) const {

      auto BEMBody = m_HDB->GetBody(m_body);

      return BEMBody->GetDiffraction(iangle, iforce);

    }

    FrLinearDiffractionForce::FrLinearDiffractionForce(const std::string& name,
                                                       const std::shared_ptr<FrHydroDB> &HDB)
        : FrLinearHDBForce(name, HDB) {}

    std::shared_ptr<FrLinearDiffractionForce>
    make_linear_diffraction_force(const std::string& name,
                                  std::shared_ptr<FrHydroDB> HDB,
                                  std::shared_ptr<FrBody> body) {

      // This function creates the linear excitation force object.

      // Construction of the excitation force object from the HDB.
      auto diffractionForce = std::make_shared<FrLinearDiffractionForce>(name, HDB);

      // Add the excitation force object as an external force to the body.
      body->AddExternalForce(diffractionForce); // Initialization of m_body.

      return diffractionForce;

    }

}  // end namespace frydom
