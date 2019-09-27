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

#include "FrLinearExcitationForce.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOceanInc.h"

namespace frydom {

    Eigen::MatrixXcd FrLinearExcitationForce::GetHDBData(unsigned int iangle) const {

      auto BEMBody = m_HDB->GetBody(m_body);

      return BEMBody->GetExcitation(iangle);

    }

    Eigen::VectorXcd FrLinearExcitationForce::GetHDBData(unsigned int iangle, unsigned int iforce) const {

      auto BEMBody = m_HDB->GetBody(m_body);

      return BEMBody->GetExcitation(iangle, iforce);

    }

    FrLinearExcitationForce::FrLinearExcitationForce(const std::string& name,
                                                     const std::shared_ptr<FrHydroDB> &HDB) :
        FrLinearHDBForce(name, HDB) {}

    std::shared_ptr<FrLinearExcitationForce>
    make_linear_excitation_force(const std::string& name,
                                 std::shared_ptr<FrHydroDB> HDB,
                                 std::shared_ptr<FrBody> body) {

      // This function creates the linear excitation force object.

      // Construction of the excitation force object from the HDB.
      auto excitationForce = std::make_shared<FrLinearExcitationForce>(name, HDB);

      // Add the excitation force object as an external force to the body.
      body->AddExternalForce(excitationForce); // Initialization of m_body.

      return excitationForce;

    }

}  // end namespace frydom
