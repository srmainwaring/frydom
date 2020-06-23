//
// Created by lletourn on 27/05/20.
//

#include "FrBEMBody_new.h"
#include "FrHydroDB_new.h"

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
  };

  std::shared_ptr<FrHydroDB> make_new_hydrodynamic_database(std::string h5file) {
    return std::make_shared<FrHydroDB>(h5file);
  }

} //end namespace frydom