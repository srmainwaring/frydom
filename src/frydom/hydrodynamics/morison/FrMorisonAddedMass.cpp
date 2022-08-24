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

#include "FrMorisonAddedMass.h"
#include "frydom/hydrodynamics/morison/FrMorisonElements.h"
#include "frydom/core/body/FrBody.h"

namespace frydom {
  namespace internal {

    FrMorisonAddedMassBase::FrMorisonAddedMassBase(FrMorisonCompositeElement *morisonModel) :
      m_frydomMorisonModel(morisonModel)
    {
      SetVar();
    }

    int FrMorisonAddedMassBase::GetNnodes() {
      return 1;
    }

    int FrMorisonAddedMassBase::GetNdofs() {
      return 6;
    }

    int FrMorisonAddedMassBase::GetNodeNdofs(int n) {
      return 6;
    }

    std::shared_ptr<chrono::fea::ChNodeFEAbase> FrMorisonAddedMassBase::GetNodeN(int n) {
      return nullptr;
    }

    void FrMorisonAddedMassBase::GetStateBlock(chrono::ChVectorDynamic<> &mD) {
      double phi, theta, psi;
      mD.setZero(this->GetNdofs());
      auto body = m_frydomMorisonModel->GetBody();
      mD.segment(0, 3) = body->GetPosition(NWU);
      mD.segment(3, 3) = Vector3d<double>(phi, theta, psi);
    }

    void FrMorisonAddedMassBase::ComputeKRMmatricesGlobal(chrono::ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
      assert(H.rows() == 6 && H.cols() == 6);
      H = m_addedMassMatrix;
    }

    void FrMorisonAddedMassBase::ComputeInternalForces(chrono::ChVectorDynamic<> &Fi) {
      Fi.setZero();
    }

    void FrMorisonAddedMassBase::ComputeGravityForces(chrono::ChVectorDynamic<> &Fi, const chrono::ChVector<> &G_acc) {
      Fi.setZero();
    }

    void FrMorisonAddedMassBase::EleIntLoadResidual_F(chrono::ChVectorDynamic<> &R, const double c) {
      // Nothing to do : No internal load
    }

    void FrMorisonAddedMassBase::EleIntLoadResidual_Mv(chrono::ChVectorDynamic<> &R, const chrono::ChVectorDynamic<> &w, const double c) {
      chrono::ChMatrixDynamic<> mMi(this->GetNdofs(), this->GetNdofs());
      this->ComputeMmatrixGlobal(mMi);

      chrono::ChVectorDynamic<> mqi(this->GetNdofs());

      auto chrono_body = internal::GetChronoBody(m_frydomMorisonModel->GetBody());

      if (chrono_body->GetBodyFixed()) {
        for (int i=0; i<6; ++i) {
          mqi(i) = 0;
        }
      } else {
        auto offset = chrono_body->GetOffset_w();
        mqi.segment(0, 6) = w.segment(offset, 6);
      }

      chrono::ChVectorDynamic<> mFi = c * mMi * mqi;

      if (!chrono_body->GetBodyFixed()) {
        auto offset = chrono_body->GetOffset_w();
        R.segment(offset, 6) += mFi.segment(0, 6);
      }
    }

    void FrMorisonAddedMassBase::EleIntLoadResidual_F_gravity(chrono::ChVectorDynamic<> &R, const chrono::ChVector<> &G_acc, const double c) {
        // Nothing to do : no gravity load
    }

    void FrMorisonAddedMassBase::Initialize() {
      m_addedMassMatrix = m_frydomMorisonModel->GetAMInBody();
    }

    void FrMorisonAddedMassBase::Update() {
      m_addedMassMatrix = m_frydomMorisonModel->GetAMInBody();
    }

    void FrMorisonAddedMassBase::SetVar() {
      std::vector<chrono::ChVariables*> mvars;
      auto chrono_body = GetChronoBody(m_frydomMorisonModel->GetBody());
      mvars.push_back(&chrono_body->Variables());
      Kmatr.SetVariables(mvars);
    }

    void FrMorisonAddedMassBase::SetupInitial(chrono::ChSystem *system) {
      m_addedMassMatrix = m_frydomMorisonModel->GetAMInBody();
    }

  } // end namespace internal
} // end namespace frydom
