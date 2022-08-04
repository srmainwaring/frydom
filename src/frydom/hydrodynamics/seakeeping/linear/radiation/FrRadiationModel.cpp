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
#include "FrRadiationModelBase.h"
#include "FrAddedMass.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "FrRadiationForce.h"

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
    //m_chronoPhysicsItem = std::make_shared<internal::FrRadiationModelBase>(this);
    m_addedMass = std::make_shared<internal::FrAddedMassBase>(this);
    auto my_mesh = std::make_shared<chrono::fea::ChMesh>();
    my_mesh->AddElement(m_addedMass);
    system->GetChronoSystem()->Add(my_mesh);
  }

  void FrRadiationModel::Initialize() {
    //FrPhysicsItem::Initialize();
    m_chronoPhysicsItem->SetupInitial();
  }

  FrHydroMapper *FrRadiationModel::GetMapper() const {
    return m_HDB->GetMapper();
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

  //##CC
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
  //##

  void FrRadiationModel::Compute(double time) {

  }

}  // end namespace frydom

