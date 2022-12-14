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


#include "FrRadiationModelBase.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrRadiationModel.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "FrVariablesBEMBodyBase.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"


namespace frydom {

  namespace internal {

    FrRadiationModelBase::FrRadiationModelBase(FrRadiationModel *radiationModel) :
        m_frydomRadiationModel(radiationModel), FrPhysicsItemBase(radiationModel) {

    }

    void FrRadiationModelBase::SetupInitial() {
      InjectVariablesToBody();
      BuildGeneralizedMass();
    }

    void FrRadiationModelBase::Update(bool update_assets) {
      this->Update(ChTime, update_assets);
    }

    void FrRadiationModelBase::Update(double time, bool update_assets) {
      m_frydomRadiationModel->Update(time);
      ChPhysicsItem::Update(time, update_assets);
    }

    void FrRadiationModelBase::IntLoadResidual_Mv(const unsigned int off, chrono::ChVectorDynamic<> &R,
                                                  const chrono::ChVectorDynamic<> &w, const double c) {

      auto HDB = m_frydomRadiationModel->GetHydroDB();

      for (auto BEMBody = HDB->begin(); BEMBody != HDB->end(); BEMBody++) {

        if (BEMBody->second->IsActive()) {

          auto residualOffset = GetBodyOffset(HDB->GetBody(BEMBody->first)); //+off

          for (auto BEMBodyMotion = HDB->begin(); BEMBodyMotion != HDB->end(); BEMBodyMotion++) {

            if (BEMBodyMotion->second->IsActive()) {

              auto bodyOffset = GetBodyOffset(HDB->GetBody(BEMBodyMotion->first)); //+off

              auto infiniteAddedMass = BEMBody->first->GetInfiniteAddedMass(BEMBodyMotion->first);

              Eigen::VectorXd q(6);
              for (int i = 0; i < 6; i++) { q(i) = w(bodyOffset + i); }
              SetInBody(q, BEMBody->first);

              Eigen::VectorXd Mv = c * infiniteAddedMass * q;
              SetInWorld(Mv, BEMBody->first);

              auto Mw = chrono::ChVector<>(Mv(0), Mv(1), Mv(2));
              auto Iw = chrono::ChVector<>(Mv(3), Mv(4), Mv(5));

              R.segment(residualOffset, 3) += Mw.eigen();
              R.segment(residualOffset + 3, 3) += Iw.eigen();
            }
          }
        }
      }
    }

    void FrRadiationModelBase::IntToDescriptor(const unsigned int off_v, const chrono::ChStateDelta &v,
                                               const chrono::ChVectorDynamic<> &R, const unsigned int off_L,
                                               const chrono::ChVectorDynamic<> &L,
                                               const chrono::ChVectorDynamic<> &Qc) {

      // Nothing to do since added mass variables encapsulate the body variables
    }

    void FrRadiationModelBase::IntFromDescriptor(const unsigned int off_v, chrono::ChStateDelta &v,
                                                 const unsigned int off_L, chrono::ChVectorDynamic<> &L) {

      // Nothing to do since added mass variables encapsulate the body variables
    }

    unsigned int FrRadiationModelBase::GetBodyOffset(FrBody *body) const {
      return internal::GetChronoBody(body)->GetOffset_w();
    }


    void FrRadiationModelBase::InjectVariablesToBody() {

      auto HDB = GetRadiationModel()->GetHydroDB();

      for (auto body = HDB->begin(); body != HDB->end(); body++) {
        auto chronoBody = internal::GetChronoBody(body->second);
        auto variable = std::make_shared<FrVariablesBEMBodyBase>(this,
                                                                 body->first,
                                                                 &chronoBody->VariablesBody());
        chronoBody->SetVariables(variable);
      }

    }

    void FrRadiationModelBase::BuildGeneralizedMass() {

      auto HDB = GetRadiationModel()->GetHydroDB();

      auto nBody = HDB->GetMapper()->GetNbMappings();

      mathutils::MatrixMN<double> massMatrix(6 * nBody, 6 * nBody);

      int iBody = 0;

      // Loop over the bodies subject to hydrodynamic loads
      for (auto body = HDB->begin(); body != HDB->end(); body++) {

        int jBody = 0;

        // Loop over the bodies subject to motion
        for (auto bodyMotion = HDB->begin(); bodyMotion != HDB->end(); bodyMotion++) {

          mathutils::Matrix66<double> subMatrix = body->first->GetInfiniteAddedMass(bodyMotion->first);

          if (bodyMotion->first == body->first) {
            subMatrix += body->second->GetInertiaTensor().GetMassMatrixAtCOG();
          }

          for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
              massMatrix(6 * iBody + i, 6 * jBody + j) = subMatrix(i, j);
            }
          }
          jBody += 1;
        }
        iBody += 1;
      }

      // Inverse the mass matrix
      massMatrix.Inverse();

      // Save the inverse of mass matrix in map
      mathutils::Matrix66<double> invMassMatrix;
      iBody = 0;
      for (auto body = HDB->begin(); body != HDB->end(); body++) {
        int jBody = 0;
        for (auto bodyMotion = HDB->begin(); bodyMotion != HDB->end(); bodyMotion++) {

          for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
              invMassMatrix(i, j) = massMatrix(6 * iBody + i, 6 * jBody + j);
            }
          }
          m_invGeneralizedMass[std::make_pair(body->first, bodyMotion->first)] = invMassMatrix;
          jBody += 1;
        }
        iBody += 1;
      }
    }


    mathutils::Matrix66<double>
    FrRadiationModelBase::GetInverseGeneralizedMass(FrBEMBody *BEMBody, FrBEMBody *BEMBodyMotion) const {
      return m_invGeneralizedMass.at(std::pair<FrBEMBody *, FrBEMBody *>(BEMBody, BEMBodyMotion));
    }

    mathutils::Matrix66<double>
    FrRadiationModelBase::GetGeneralizedMass(FrBEMBody *BEMBody, FrBEMBody *BEMBodyMotion) const {
      auto generalizedMass = BEMBody->GetInfiniteAddedMass(BEMBodyMotion);

      if (BEMBody == BEMBodyMotion) {
        auto body = GetRadiationModel()->GetHydroDB()->GetMapper()->GetBody(BEMBody);
        generalizedMass += body->GetInertiaTensor().GetMassMatrixAtCOG();
      }

      return generalizedMass;
    }

    void FrRadiationModelBase::SetInBody(Eigen::VectorXd &vect, hdb5_io::Body* body) {

      auto eq_frame = m_frydomRadiationModel->GetHydroDB()->GetMapper()->GetEquilibriumFrame(body);
      auto v_loc = eq_frame->GetFrame().ProjectVectorParentInFrame(Vector3d(vect(0), vect(1), vect(2)), NWU);
      for (int i=0; i<3; i++) vect[i] = v_loc[i];

    }

    void FrRadiationModelBase::SetInWorld(Eigen::VectorXd &vect, hdb5_io::Body* body) {

      auto eq_frame = m_frydomRadiationModel->GetHydroDB()->GetMapper()->GetEquilibriumFrame(body);
      auto v_world = eq_frame->GetFrame().ProjectVectorFrameInParent(Vector3d(vect(0), vect(1), vect(2)), NWU);
      for (int i=0; i<3; i++) vect[i] = v_world[i];

    }

  }  // end namespace frydom::internal

}  // end namespace frydom
