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

#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrAddedMass.h"
#include "frydom/core/math/FrMatrix.h"
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrRadiationModel.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/cable/fea/FrFEANode.h"
#include "frydom/cable/fea/FrFEALink.h"

namespace frydom {

  // --------------------------------------------------------------------------------
  // FrAddedMassBase
  // --------------------------------------------------------------------------------

  namespace internal {

    FrAddedMassBase::FrAddedMassBase(FrRadiationModel* radiationModel)
      : m_frydomRadiationModel(radiationModel), m_is_active(true) {
      m_hdb = m_frydomRadiationModel->GetHydroDB();
      SetNodes();
    }

    FrAddedMassBase::~FrAddedMassBase() { }

    int FrAddedMassBase::GetNnodes() { return m_hdb->GetBEMBodyNumber(); }

    int FrAddedMassBase::GetNdofs() { return 6 * m_nb_bodies; }

    int FrAddedMassBase::GetNodeNdofs(int n) { return 6; }

    std::shared_ptr<chrono::fea::ChNodeFEAbase> FrAddedMassBase::GetNodeN(int n) {
      return nullptr;
    }

    void FrAddedMassBase::GetStateBlock(chrono::ChVectorDynamic<>& mD) {
      double phi, theta, psi;
      mD.setZero(this->GetNdofs());
      int iBody = 0;
      for (auto body = m_hdb->begin(); body != m_hdb->end(); body++) {
        mD.segment(6*iBody, 3) = m_hdb->GetBody(body->first)->GetPosition(NWU);
        m_hdb->GetBody(body->first)->GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, NWU);
        mD.segment(6*iBody+3, 3) = Vector3d<double>(phi, theta, psi);
        iBody++;
      }
    }

    void FrAddedMassBase::ComputeKRMmatricesGlobal(chrono::ChMatrixRef H, double Kfactor, double Rfactor,
                                                   double Mfactor) {
      assert((H.rows() == 6 * m_nb_bodies) && (H.cols() == 6 * m_nb_bodies));

      if (!m_is_active) {
        H.setZero();
      } else {
        //##CC
        BuildGeneralizedMass();
        H = m_addedMassMatrixInWorld;
        //##
        //##CC H = m_addedMassMatrix;
      }
    }

    void FrAddedMassBase::ComputeInternalForces(chrono::ChVectorDynamic<>& Fi) {
      Fi.setZero();
    }

    void FrAddedMassBase::ComputeGravityForces(chrono::ChVectorDynamic<>& Fi, const chrono::ChVector<>& G_acc) {
      Fi.setZero();
    }

    void FrAddedMassBase::BuildGeneralizedMass() {

      auto n_body = m_hdb->GetMapper()->GetNbMappings();
      m_nb_bodies = n_body;

      m_addedMassMatrix = mathutils::MatrixMN<double>(6 * n_body, 6 * n_body);
      //##CC
      m_addedMassMatrixInWorld = mathutils::MatrixMN<double>(6 * n_body, 6 * n_body);
      //##

      int iBody = 0;

      // Loop over the bodies subject to hydrodynamic loads
      for (auto body = m_hdb->begin(); body != m_hdb->end(); body++) {
        int jBody = 0;
        // Loop over the bodies subject to motion
        for (auto bodyMotion = m_hdb->begin(); bodyMotion != m_hdb->end(); bodyMotion++) {

          mathutils::Matrix66<double> subMatrix = body->first->GetInfiniteAddedMass(bodyMotion->first);

          //##CC debug
          mathutils::Matrix66<double> subMatrixInWorld = body->first->GetInfiniteAddedMass(bodyMotion->first);
          auto matRot = body->second->GetFrame().GetRotation().GetRotationMatrix();
          subMatrixInWorld.block<3, 3>(0, 0) = matRot * subMatrixInWorld.block<3, 3>(0, 0) * matRot.transpose();
          subMatrixInWorld.block<3, 3>(0, 3) = matRot * subMatrixInWorld.block<3, 3>(0, 3) * matRot.transpose();
          subMatrixInWorld.block<3, 3>(3, 0) = matRot * subMatrixInWorld.block<3, 3>(3, 0) * matRot.transpose();
          subMatrixInWorld.block<3, 3>(3, 3) = matRot * subMatrixInWorld.block<3, 3>(3, 3) * matRot.transpose();
          //##

          for (int i=0; i<6; i++) {
            for (int j=0; j<6; j++) {
              m_addedMassMatrix(6 * iBody + i, 6 * jBody + j) = subMatrix(i, j);
              //##CC debug
              m_addedMassMatrixInWorld(6 * iBody + i, 6 * jBody + j) = subMatrixInWorld(i, j);
              //##
            }
          }
          m_generalizedAddedMass[std::make_pair(body->first, bodyMotion->first)] = subMatrix;
          //##CC
          m_generalizedAddedMassInWorld[std::make_pair(body->first, bodyMotion->first)] = subMatrixInWorld;
          //##
          jBody += 1;
        }
        iBody += 1;
      }
    }

    void FrAddedMassBase::EleIntLoadResidual_F(chrono::ChVectorDynamic<>& R, const double c) {

      if (!m_is_active) {
        return;
      }

      chrono::ChVectorDynamic<> mFi(this->GetNdofs());
      this->ComputeInternalForces(mFi);
      mFi *= c;

      //// RADU
      //// Attention: this is called from within a parallel OMP for loop.
      //// Must use atomic increment when updating the global vector R.

      int stride = 0;
      for (auto body = m_hdb->begin(); body != m_hdb->end(); body++) {
        auto chrono_body = internal::GetChronoBody(m_hdb->GetBody(body->first));
        if (!chrono_body->GetBodyFixed()) {
          for (int j = 0; j < 6; j++)
              #pragma omp atomic
              R(chrono_body->GetOffset_w() + j) += mFi(stride + j);
        }
        stride += 6;
      }
    }

    void FrAddedMassBase::EleIntLoadResidual_Mv(chrono::ChVectorDynamic<>& R,
                                                const chrono::ChVectorDynamic<>& w,
                                                const double c) {

      if (!m_is_active) {
        return;
      }

      chrono::ChMatrixDynamic<> mMi(this->GetNdofs(), this->GetNdofs());
      //##CC this->ComputeMmatrixGlobal(mMi);
      mMi = m_addedMassMatrix;
      //##

      chrono::ChVectorDynamic<> mqi(this->GetNdofs());
      int stride = 0;

      for (auto body = m_hdb->begin(); body != m_hdb->end(); body++) {
        auto chrono_body = internal::GetChronoBody(m_hdb->GetBody(body->first));
        if (chrono_body->GetBodyFixed()) {
          for (int i =0; i < 6; ++i)
            mqi(stride + i) = 0;
        } else {
          //##CC auto offset = chrono_body->GetOffset_w();
          //##CC mqi.segment(stride, 6) = w.segment(offset, 6);
          //##CC
          auto offset = chrono_body->GetOffset_w();
          Vector3d pos_in_world = w.segment(offset, 3);
          Vector3d rot_in_world = w.segment(offset+3, 3);
          auto pos_in_body = body->second->GetFrame().ProjectVectorParentInFrame(pos_in_world, NWU);
          auto rot_in_body = body->second->GetFrame().ProjectVectorParentInFrame(rot_in_world, NWU);
          mqi.segment(stride, 3) = pos_in_body;
          mqi.segment(stride+3, 3) = rot_in_body;
          //##
        }
        stride += 6;
      }

      chrono::ChVectorDynamic<> mFi = c  *mMi * mqi;

      stride = 0;
      for (auto body = m_hdb->begin(); body != m_hdb->end(); body++) {
        auto chrono_body = internal::GetChronoBody(m_hdb->GetBody(body->first));
        if (!chrono_body->GetBodyFixed()) {
          //##CC auto offset = chrono_body->GetOffset_w();
          //##CC R.segment(offset, 6) += mFi.segment(stride, 6);
          //##CC
          auto offset = chrono_body->GetOffset_w();
          Vector3d mFi_in_body = mFi.segment(stride, 3);
          Vector3d mMi_in_body = mFi.segment(stride+3, 3);
          auto Fi_in_world = body->second->GetFrame().ProjectVectorFrameInParent(mFi_in_body, NWU);
          auto Mi_in_world = body->second->GetFrame().ProjectVectorFrameInParent(mMi_in_body, NWU);
          R.segment(offset, 3) += Fi_in_world;
          R.segment(offset+3, 3) += Mi_in_world;
          //##
        }
        stride += 6;
      }
    }

    void FrAddedMassBase::EleIntLoadResidual_F_gravity(chrono::ChVectorDynamic<>& R, const chrono::ChVector<>& G_acc, const double c) {

      if (!m_is_active) {
        return;
      }

      chrono::ChVectorDynamic<> mFg(this->GetNdofs());
      this->ComputeGravityForces(mFg, G_acc);
      mFg *= c;

      int stride = 0;
      //for (int in = 0; in < this->GetNnodes(); in++) {
      for (auto body = m_hdb->begin(); body != m_hdb->end(); body++) {
        auto chrono_body = internal::GetChronoBody(m_hdb->GetBody(body->first));
        if (!chrono_body->GetBodyFixed()) {
          for (int j = 0; j < 6; j++)
            //// ATOMIC as called from an OMP parallel loop: this is here to avoid race conditions when writing to R
              #pragma omp atomic
              R(chrono_body->GetOffset_w() + j) += mFg(stride + j);
        }
        stride += 6;
      }
    }

    void FrAddedMassBase::SetNodes() {

      std::vector<chrono::ChVariables*> mvars;

      for (auto body = m_hdb->begin(); body != m_hdb->end(); body++) {

        auto chrono_body = GetChronoBody(m_hdb->GetBody(body->first));

        mvars.push_back(&chrono_body->Variables());

        m_bodies.push_back(chrono_body);
      }
      Kmatr.SetVariables(mvars);
    }

    void FrAddedMassBase::Initialize() {
      BuildGeneralizedMass();
    }

    void FrAddedMassBase::SetActive(bool is_active) {
      m_is_active = is_active;
    }

    void FrAddedMassBase::SetupInitial(chrono::ChSystem* system) {
      BuildGeneralizedMass();
    }

  } // end namespace internal

} // end namespace frydom

