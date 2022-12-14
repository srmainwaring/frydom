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

#include "FrBEMBody.h"
#include "FrHydroDB.h"
#include "frydom/core/body/FrBody.h"

namespace frydom {


  FrHydroDB::FrHydroDB(std::string h5file) {

    m_HDB = hdb5_io::import_HDB(h5file);

    // Creation of the mapper object for hydrodynamic bodies.
    m_mapper = std::make_unique<FrHydroMapper>();

  }

  FrHydroDB::FrHydroDB(const std::shared_ptr<hdb5_io::HydrodynamicDataBase> &hdb) : m_HDB(hdb) {

    // Creation of the mapper object for hydrodynamic bodies.
    m_mapper = std::make_unique<FrHydroMapper>();

  };

  FrBEMBody *FrHydroDB::GetBody(FrBody *body) {
    return m_mapper->GetBEMBody(body);
  }

  FrBEMBody *FrHydroDB::GetBody(int ibody) {
    return m_HDB->GetBody(ibody);
  }

  FrBody *FrHydroDB::GetBody(FrBEMBody *body) {
    return m_mapper->GetBody(body);
  }

  int FrHydroDB::GetBEMBodyNumber() {

    // This method returns the number of BEM bodies.

    return m_HDB->GetNbBodies();

  }

  FrHydroMapper *FrHydroDB::GetMapper() {
    return m_mapper.get();
  }

  void FrHydroDB::Map(FrBEMBody *BEMBody, FrBody *body, std::shared_ptr<FrEquilibriumFrame> eqFrame) {
    m_mapper->Map(BEMBody, body, eqFrame);
  }

  void FrHydroDB::Map(int iBEMBody, FrBody *body, std::shared_ptr<FrEquilibriumFrame> eqFrame) {
    m_mapper->Map(m_HDB->GetBody(iBEMBody), body, eqFrame);
  }

  std::unordered_map<FrBEMBody *, FrBody *>::iterator FrHydroDB::begin() {
    return m_mapper->begin();
  };

  std::unordered_map<FrBEMBody *, FrBody *>::iterator FrHydroDB::end() {
    return m_mapper->end();
  }

  FrWaveDriftPolarData * FrHydroDB::GetWaveDrift() const {
    return m_HDB->GetWaveDrift();
  }

  double FrHydroDB::GetMinFrequency() const {
    return m_HDB->GetMinFrequency();
  }

  double FrHydroDB::GetMaxFrequency() const {
    return m_HDB->GetMaxFrequency();
  }

  FrMask FrHydroDB::GetBodyDOFMask(FrBEMBody *BEMBody) const {

    auto mask = BEMBody->GetForceMask();

    if (m_dofMaskApplied) {
      hdb5_io::Mask DOFMask;
      DOFMask.SetMask(m_mapper->GetBody(BEMBody)->GetDOFMask()->GetFreeDOFs());
      mask = mask&&DOFMask;
    }

    return mask;
  }

  mathutils::Matrix66<bool> FrHydroDB::GetBodyRadiationMask(FrBEMBody* BEMBody, FrBEMBody* BEMBodyMotion) {
    auto mask = BEMBody->GetRadiationMask(BEMBodyMotion);
    if (m_dofMaskApplied) {
      // Applying the BEMBody DOFMask also on the radiationMask to ensure all forces on locked dofs are zero
      hdb5_io::Mask DOFMask;
      DOFMask.SetMask(this->GetBody(BEMBody)->GetDOFMask()->GetLockedDOFs());
      for (auto idof : DOFMask.GetListDOF()) {
        mask.row(idof) *= false;
      }
    }
    return mask;
  }

  bool FrHydroDB::GetIsXDerivative() const {

    // This method gives the boolean to known if x-derivatives of the added mass and damping coefficients are present.

    return m_HDB->GetIsXDerivative();

  }

  std::shared_ptr<FrHydroDB> make_hydrodynamic_database(std::string h5file) {
    return std::make_shared<FrHydroDB>(h5file);
  }

  std::shared_ptr<FrHydroDB> make_hydrodynamic_database(const std::shared_ptr<hdb5_io::HydrodynamicDataBase> &hdb) {
    return std::make_shared<FrHydroDB>(hdb);
  }

} //end namespace frydom
