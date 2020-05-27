//
// Created by lletourn on 27/05/20.
//

#include "FrHydroMapper_new.h"

namespace frydom {

  void FrHydroMapper_new::Map(FrBEMBody_new *BEMBody, FrBody *body, std::shared_ptr<FrEquilibriumFrame> eqFrame) {
    m_mapBEMToBody[BEMBody] = body;
    m_mapBodyToBEM[body] = BEMBody;
    m_mapEqFrame[BEMBody] = eqFrame;
  }

  unsigned long FrHydroMapper_new::GetNbMappings() const {
    return m_mapBEMToBody.size();
  }

  FrBody *FrHydroMapper_new::GetBody(FrBEMBody_new *BEMBody) const {
    return m_mapBEMToBody.at(BEMBody);
  }

  FrBEMBody_new *FrHydroMapper_new::GetBEMBody(FrBody *body) const {
    return m_mapBodyToBEM.at(body);
  }

  unsigned int FrHydroMapper_new::GetBEMBodyIndex(FrBody *body) const {
    auto BEMBody = m_mapBodyToBEM.at(body);
    return BEMBody->GetID();
  }

  FrEquilibriumFrame *FrHydroMapper_new::GetEquilibriumFrame(FrBEMBody_new *BEMBody) const {
    return m_mapEqFrame.at(BEMBody).get();
  }

  FrEquilibriumFrame *FrHydroMapper_new::GetEquilibriumFrame(FrBody *body) const {
    auto BEMBody = this->GetBEMBody(body);
    return m_mapEqFrame.at(BEMBody).get();
  }

  std::shared_ptr<FrEquilibriumFrame> FrHydroMapper_new::GetSharedEquilibriumFrame(FrBody *body) const {
    return m_mapEqFrame.at(GetBEMBody(body));
  }

  std::unordered_map<FrBEMBody_new *, FrBody *>::iterator FrHydroMapper_new::begin() {
    return m_mapBEMToBody.begin();
  }

  std::unordered_map<FrBEMBody_new *, FrBody *>::iterator FrHydroMapper_new::end() {
    return m_mapBEMToBody.end();
  }

} //end namespace frydom
