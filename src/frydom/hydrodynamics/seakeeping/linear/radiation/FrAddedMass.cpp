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
      : m_frydomRadiationModel(radiationModel) {
      SetNodes();
    }

    FrAddedMassBase::~FrAddedMassBase() { }

    int FrAddedMassBase::GetNnodes() { return m_nodes.size(); }

    int FrAddedMassBase::GetNdofs() { return 6 * m_nb_bodies; }

    int FrAddedMassBase::GetNodeNdofs(int n) { return 6; }

    std::shared_ptr<chrono::fea::ChNodeFEAbase> FrAddedMassBase::GetNodeN(int n) {
      return m_nodes[n];
      //##CC debug
      //m_bodies[n]->SetOffset_w(6);
      //auto node = std::make_shared<FrFEANodeBase>(*m_bodies[n]);
      //std::cout << "debug : FrAddedMassBase : body offset  = " << m_bodies[n]->GetOffset_w() << std::endl;
      //std::cout << "debug : FrAddedMassBase : node offset  = " << node->NodeGetOffset_w() << std::endl;
      //##
      //return std::make_shared<FrFEANodeBase>(*m_bodies[n]);
    }

    void FrAddedMassBase::GetStateBlock(chrono::ChVectorDynamic<>& mD) {
      auto hdb = GetRadiationModel()->GetHydroDB();
      double phi, theta, psi;
      mD.setZero(this->GetNdofs());
      int iBody = 0;
      for (auto body = hdb->begin(); body != hdb->end(); body++) {
        mD.segment(6*iBody, 3) = hdb->GetBody(body->first)->GetPosition(NWU);
        hdb->GetBody(body->first)->GetRotation().GetCardanAngles_RADIANS(phi, theta, psi, NWU);
        mD.segment(6*iBody+3, 3) = Vector3d<double>(phi, theta, psi);
        iBody++;
      }
    }

    void FrAddedMassBase::ComputeKRMmatricesGlobal(chrono::ChMatrixRef H, double Kfactor, double Rfactor,
                                                   double Mfactor) {
      assert((H.rows() == 6 * m_nb_bodies) && (H.cols() == 6 * m_nb_bodies));
      H = m_addedMassMatrix;
    }

    void FrAddedMassBase::ComputeInternalForces(chrono::ChVectorDynamic<>& Fi) {
      Fi.setZero();
    }

    void FrAddedMassBase::ComputeGravityForces(chrono::ChVectorDynamic<>& Fi, const chrono::ChVector<>& G_acc) {
      Fi.setZero();
    }

    void FrAddedMassBase::BuildGeneralizedMass() {

      auto hdb = m_frydomRadiationModel->GetHydroDB();

      auto n_body = hdb->GetMapper()->GetNbMappings();
      m_nb_bodies = n_body;

      m_addedMassMatrix = mathutils::MatrixMN<double>(6 * n_body, 6 * n_body);

      int iBody = 0;

      // Loop over the bodies subject to hydrodynamic loads
      for (auto body = hdb->begin(); body != hdb->end(); body++) {
        int jBody = 0;
        // Loop over the bodies subject to motion
        for (auto bodyMotion = hdb->begin(); bodyMotion != hdb->end(); bodyMotion++) {

          mathutils::Matrix66<double> subMatrix = body->first->GetInfiniteAddedMass(bodyMotion->first);

          for (int i=0; i<6; i++) {
            for (int j=0; j<6; j++) {
              m_addedMassMatrix(6 * iBody + i, 6 * jBody + j) = subMatrix(i, j);
            }
          }
          m_generalizedAddedMass[std::make_pair(body->first, bodyMotion->first)] = subMatrix;
          jBody += 1;
        }
        iBody += 1;
      }
    }


    void FrAddedMassBase::EleIntLoadResidual_Mv(chrono::ChVectorDynamic<>& R,
                                                const chrono::ChVectorDynamic<>& w,
                                                const double c) {

      auto hdb = GetRadiationModel()->GetHydroDB();

      chrono::ChMatrixDynamic<> mMi(this->GetNdofs(), this->GetNdofs());
      this->ComputeMmatrixGlobal(mMi);

      chrono::ChVectorDynamic<> mqi(this->GetNdofs());
      int stride = 0;
      for (int in = 0; in < this->GetNnodes(); in++) {
        int nodedofs = GetNodeNdofs(in);
        if (GetNodeN(in)->GetFixed()) {
          for (int i =0; i < nodedofs; ++i)
            mqi(stride + i) = 0;
        } else {
          auto offset = GetNodeN(in)->NodeGetOffset_w();
          mqi.segment(stride, nodedofs) = w.segment(offset, nodedofs);
        }
        stride += nodedofs;
      }

      chrono::ChVectorDynamic<> mFi = c  *mMi * mqi;

      stride = 0;
      for (int in = 0; in < this->GetNnodes(); in++) {
        int nodedofs = GetNodeNdofs(in);
        if (!GetNodeN(in)->GetFixed()) {
          //R.segment(internal::GetChronoBody(hdb->GetBody(hdb->GetBody(in)))->GetOffset_w(), nodedofs) +=
          //    mFi.segment(stride, nodedofs);
            auto offset = GetNodeN(in)->NodeGetOffset_w();
          R.segment(offset, nodedofs) += mFi.segment(stride, nodedofs);
        }
        stride += nodedofs;
      }
    }


    void FrAddedMassBase::SetNodes() {

      auto hdb = m_frydomRadiationModel->GetHydroDB();

      std::vector<chrono::ChVariables*> mvars;

      for (auto body = hdb->begin(); body != hdb->end(); body++) {

        auto chrono_body = GetChronoBody(hdb->GetBody(body->first));

        auto node = std::make_shared<FrFEANodeBase>(*chrono_body);
        auto link = std::make_shared<internal::FrFEALinkBase>("link_"+body->first->GetName()+"_Ma",
                                                              m_frydomRadiationModel->GetSystem());
        link->Initialize(node, chrono_body, true,
                         chrono::ChFrame<double>(),chrono::ChFrame<double>());

        m_nodes.push_back(node);
        m_links.push_back(link);
        mvars.push_back(&node->Variables());

        //m_nodes.push_back(std::make_shared<internal::FrFEANodeBase>(chrono_body.get()));
        //m_nodes.push_back(std::make_shared<FrFEANodeBase>(chrono::ChFrame<>()));

        m_bodies.push_back(chrono_body);
      }
      Kmatr.SetVariables(mvars);
    }

    void FrAddedMassBase::Initialize() {
      BuildGeneralizedMass();
    }

    void FrAddedMassBase::SetupInitial(chrono::ChSystem* system) {
      BuildGeneralizedMass();
      //SetNodes();
    }

  } // end namespace internal

  // --------------------------------------------------------------------------------
  // FrAddedMass
  // --------------------------------------------------------------------------------

} // end namespace frydom

