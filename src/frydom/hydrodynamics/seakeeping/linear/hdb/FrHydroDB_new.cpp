//
// Created by lletourn on 27/05/20.
//

#include "FrHydroDB_new.h"

namespace frydom {


  FrHydroDB_new::FrHydroDB_new(std::string h5file) {

    m_HDB = HDB5_io::import_HDB(h5file);

    // Creation of the mapper object for hydrodynamic bodies.
    m_mapper = std::make_unique<FrHydroMapper_new>();

  }

  FrHydroMapper_new *FrHydroDB_new::GetMapper() {
    return m_mapper.get();
  }

  void FrHydroDB_new::Map(FrBEMBody_new *BEMBody, FrBody *body, std::shared_ptr<FrEquilibriumFrame> eqFrame) {
    m_mapper->Map(BEMBody, body, eqFrame);
  }

  void FrHydroDB_new::Map(int iBEMBody, FrBody *body, std::shared_ptr<FrEquilibriumFrame> eqFrame) {
    m_mapper->Map(m_HDB->GetBody(iBEMBody), body, eqFrame);
  }

  std::unordered_map<FrBEMBody_new *, FrBody *>::iterator FrHydroDB_new::begin() {
    return m_mapper->begin();
  };

  std::unordered_map<FrBEMBody_new *, FrBody *>::iterator FrHydroDB_new::end() {
    return m_mapper->end();
  };

  std::shared_ptr<FrHydroDB_new> make_new_hydrodynamic_database(std::string h5file) {
    return std::make_shared<FrHydroDB_new>(h5file);
  }

} //end namespace frydom