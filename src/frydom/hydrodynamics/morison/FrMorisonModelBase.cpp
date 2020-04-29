//
// Created by camille on 16/04/2020.
//

#include "FrMorisonModelBase.h"

#include "frydom/core/common/FrVariablesAddedMass.h"
#include "frydom/hydrodynamics/morison/FrMorisonElements.h"
#include "frydom/core/body/FrBody.h"

namespace frydom {

  namespace internal {

    FrMorisonModelBase::FrMorisonModelBase(FrMorisonCompositeElement* morisonCompositeElement) :
      m_frydomMorisonCompositeElement(morisonCompositeElement), FrPhysicsItemBase(morisonCompositeElement) {

    }

    void FrMorisonModelBase::SetupInitial() {
      auto body = m_frydomMorisonCompositeElement->GetBody();
      mathutils::Matrix66<double> added_mass;
      added_mass.setZero();
      m_variables = std::make_shared<FrVariablesAddedMass>(added_mass, body);
    }

    void FrMorisonModelBase::Update(bool update_assets) {
      this->Update(ChTime, update_assets);
    }

    void FrMorisonModelBase::Update(double time, bool update_assets) {
      //m_frydomMorisonCompositeElement->Update();
      ChPhysicsItem::Update(time, update_assets);

      mathutils::Matrix66<double> added_mass;
      added_mass << m_frydomMorisonCompositeElement->GetAMInBody().block<6, 3>(0, 0),
                    m_frydomMorisonCompositeElement->GetAMInBody().block<6, 3>(0, 3);

      m_variables->SetAddedMass(added_mass);
    }

    void FrMorisonModelBase::IntLoadResidual_Mv(const unsigned int off, chrono::ChVectorDynamic<>& R,
                                                const chrono::ChVectorDynamic<>& w, const double c) {

      auto bodyOffset = GetBodyOffset();
      mathutils::Matrix66<double> added_mass = m_variables->GetAddedMass();

      Eigen::VectorXd q(6);
      for (int i=0; i<6; ++i) {
        q(i) = w(bodyOffset + i);
      }

      Eigen::VectorXd Mv = c* added_mass * q;
      auto Mw = chrono::ChVector<>(Mv(0), Mv(1), Mv(2));
      auto Iw = chrono::ChVector<>(Mv(3), Mv(4), Mv(5));

      R.PasteSumVector(Mw, bodyOffset, 0);
      R.PasteSumVector(Iw, bodyOffset+3, 0);
    }

    int FrMorisonModelBase::GetBodyOffset() const {
      auto chronoBody = m_frydomMorisonCompositeElement->GetBody()->GetChronoBody();
      return chronoBody->GetOffset_w();
    }

    void FrMorisonModelBase::IntToDescriptor(const unsigned int off_v, const chrono::ChStateDelta &v,
                                             const chrono::ChVectorDynamic<>& R, const unsigned int off_L,
                                             const chrono::ChVectorDynamic<>& L,
                                             const chrono::ChVectorDynamic<> &Qc) {
      // Nothing to do since added mass encapsulate the body variables
    }

    void FrMorisonModelBase::IntFromDescriptor(const unsigned int off_v, chrono::ChStateDelta &v,
                                               const unsigned int off_L, chrono::ChVectorDynamic<> &L) {
      // Nothing to do since added mass variables encapsulate the body variables
    }

  } // end namespace internal
} // end namespace frydom
