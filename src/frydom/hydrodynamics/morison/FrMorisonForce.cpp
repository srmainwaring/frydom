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


#include "FrMorisonForce.h"

#include "frydom/hydrodynamics/morison/FrMorisonElements.h"
#include "frydom/core/common/FrVariablesAddedMass.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/core/common/FrNode.h"
#include "frydom/logging/FrTypeNames.h"

namespace frydom {

  //FrMorisonSingleElement *FrMorisonForce::SetSingleElementModel(FrBody *body) {
  //  m_model = std::make_shared<FrMorisonSingleElement>(body);
  //  return dynamic_cast<FrMorisonSingleElement *>(m_model.get());
  //}

  //FrMorisonCompositeElement *FrMorisonForce::SetCompositeElementModel(FrBody *body) {
  //  m_model = std::make_shared<FrMorisonCompositeElement>(body);
  //  return dynamic_cast<FrMorisonCompositeElement *>(m_model.get());
  //}

  FrMorisonForce::FrMorisonForce(const std::string &name, FrBody *body, std::shared_ptr<FrMorisonElement> model)
      : FrForce(name, TypeToString(this), body), m_model(model) {}

  void FrMorisonForce::Compute(double time) {

    //m_model->Update(time);

    SetForceInWorldAtCOG(m_model->GetForceInWorld(NWU), NWU);
    SetTorqueInBodyAtCOG(m_model->GetTorqueInBody(), NWU);

    //if (m_model->IsExtendedModel()) {
    //
    //  mathutils::Matrix66<double> added_mass;
    //  added_mass << m_model->GetAMInBody().block<6, 3>(0, 0),
    //                m_model->GetAMInBody().block<6, 3>(0, 3);
    //
    //  m_variables->SetAddedMass(added_mass);
    //}
  }

  void FrMorisonForce::Initialize() {

    FrForce::Initialize();
    //m_model->Initialize();

    //if (m_model->IsExtendedModel()) {
      //mathutils::Matrix66<double> added_mass;
      //added_mass.setZero();
      //added_mass << m_model->GetAMInBody().block<6, 3>(0, 0),
      //    m_model->GetAMInBody().block<6, 3>(0, 3);
      //added_mass.block<6, 3>(0, 0) = m_model->GetAMInBody().block<6, 3>(0, 0);
      //m_variables = std::make_shared<internal::FrVariablesAddedMass>(
      //    added_mass, GetBody());
    //}

  }

  std::shared_ptr<FrMorisonForce>
  make_morison_force(const std::string &name,
                     std::shared_ptr<FrBody> body,
                     std::shared_ptr<FrMorisonElement> model) {

    assert(body.get() == model->GetNode()->GetBody());
    auto MorisonForce = std::make_shared<FrMorisonForce>(name, body.get(), model);
    body->AddExternalForce(MorisonForce);
    return MorisonForce;
  }

}  // end namespace frydom
