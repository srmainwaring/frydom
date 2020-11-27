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

    m_HDB = HDB5_io::import_HDB(h5file);

    // Creation of the mapper object for hydrodynamic bodies.
    m_mapper = std::make_unique<FrHydroMapper>();

  }

  FrBEMBody *FrHydroDB::GetBody(FrBody *body) {
    return m_mapper->GetBEMBody(body);
  }

  FrBEMBody *FrHydroDB::GetBody(int ibody) {
    return m_HDB->GetBody(ibody);
  }

  FrBody *FrHydroDB::GetBody(FrBEMBody *body) {
    return m_mapper->GetBody(body);
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

  FrMask FrHydroDB::GetBodyDOFMask(FrBEMBody *BEMBody) const {

    auto DOFMask = BEMBody->GetForceMask(); // just to get the correct class type...
    //##CC - Comment to force computation of force
    //DOFMask.SetMask(m_mapper->GetBody(BEMBody)->GetDOFMask()->GetFreeDOFs());
    //##

    return DOFMask&&BEMBody->GetForceMask();
  }

  void FrHydroDB::GetImpulseResponseSize(double timeStep, double &Te, double &dt) const {

    // FIXME : check this
    auto frequencies = m_HDB->GetFrequencyDiscretization();
    auto freqStep = frequencies[1] - frequencies[0];

    Te = 0.5 * MU_2PI / freqStep;

    auto N = (unsigned int) floor(Te / timeStep);

    dt = Te / double(N - 1);
  };

  std::shared_ptr<FrHydroDB> make_hydrodynamic_database(std::string h5file) {
    return std::make_shared<FrHydroDB>(h5file);
  }

} //end namespace frydom