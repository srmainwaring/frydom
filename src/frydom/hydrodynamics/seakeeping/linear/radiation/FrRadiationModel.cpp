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

#include "FrRadiationModel.h"
#include "FrRadiationModelBaseVariable.h"
#include "FrRadiationModelBaseKRM.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrRadiationForce.h"

namespace frydom {

  // ----------------------------------------------------------------
  // Radiation model
  // ----------------------------------------------------------------

  FrRadiationModel::FrRadiationModel(const std::string &name,
                                     FrOffshoreSystem *system,
                                     std::shared_ptr<FrHydroDB> HDB) :
      FrTreeNode(name, system),
      m_HDB(HDB) {

    // Creation of an AddedMassBase object.
    m_chronoAddedMassModel = std::make_shared<internal::FrRadiationModelBaseKRM>(this);
  }

  void FrRadiationModel::Initialize() {
    //FrPhysicsItem::Initialize();
    m_chronoPhysicsItem->SetupInitial();
    if (m_chronoAddedMassModel) {
      m_chronoAddedMassModel->SetupInitial();
    }
  }

  FrHydroMapper *FrRadiationModel::GetMapper() const {
    return m_HDB->GetMapper();
  }

  void FrRadiationModel::SetActive(bool is_active) {
    m_isActive = is_active;
    if (m_chronoAddedMassModel) {
      m_chronoAddedMassModel->GetAddedMass()->SetActive(is_active);
    }
    for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); ++BEMBody) {
      for (auto &force : BEMBody->second->GetForceList()) {
        if (auto force_rad = std::dynamic_pointer_cast<FrRadiationForce>(force)) {
          force_rad->SetActive(is_active);
        }
      }
    }
  }

  Force FrRadiationModel::GetRadiationForce(FrBEMBody *BEMBody) const {
    return m_radiationForce.at(BEMBody).GetForce();
  }

  Force FrRadiationModel::GetRadiationForce(FrBody *body) const {
    auto BEMBody = m_HDB->GetBody(body);
    return m_radiationForce.at(BEMBody).GetForce();
  }

  Torque FrRadiationModel::GetRadiationTorque(FrBEMBody *BEMBody) const {
    return m_radiationForce.at(BEMBody).GetTorque();
  }

  Torque FrRadiationModel::GetRadiationTorque(FrBody *body) const {
    auto BEMBody = m_HDB->GetBody(body);
    return m_radiationForce.at(BEMBody).GetTorque();
  }

  GeneralizedForce FrRadiationModel::GetRadiationInertiaPart(FrBody *body) const {

    auto HDB = GetHydroDB();
    auto BEMBody = HDB->GetBody(body);

    auto force = GeneralizedForce();

    for (auto BEMBodyMotion = HDB->begin(); BEMBodyMotion != HDB->end(); BEMBodyMotion++) {

      auto infiniteAddedMass = BEMBody->GetInfiniteAddedMass(BEMBodyMotion->first);
      auto acc = GeneralizedAcceleration(BEMBodyMotion->second->GetCOGAccelerationInBody(NWU),
                                         BEMBodyMotion->second->GetAngularAccelerationInBody(NWU));
      force += -infiniteAddedMass * acc;
    }

    return force;
  }

  GeneralizedForce FrRadiationModel::GetRadiationSteadyInertiaPart(FrBody* body) const {

    auto HDB = GetHydroDB();
    auto BEMBody = HDB->GetBody(body);

    auto loc_gen_force = GeneralizedForce();

    for (auto BEMBodyMotion = HDB->begin(); BEMBodyMotion != HDB->end(); BEMBodyMotion++) {

      auto infiniteAddedMass = BEMBody->GetInfiniteAddedMass(BEMBodyMotion->first);
      auto acc = HDB->GetMapper()->GetEquilibriumFrame(BEMBodyMotion->first)->GetGeneralizedAccelerationInFrame(NWU);

      loc_gen_force += infiniteAddedMass * acc;

    }

    auto frame = HDB->GetMapper()->GetEquilibriumFrame(BEMBody)->GetFrame();
    auto force = frame.ProjectVectorFrameInParent(loc_gen_force.GetForce(), NWU);
    auto torque = frame.ProjectVectorFrameInParent(loc_gen_force.GetTorque(), NWU);

    return GeneralizedForce(force, torque);

  }

  void FrRadiationModel::Compute(double time) {

  }

}  // end namespace frydom

