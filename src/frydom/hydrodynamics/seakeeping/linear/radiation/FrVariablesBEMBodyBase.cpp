//
// Created by camille on 15/05/19.
//

#include "FrVariablesBEMBodyBase.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
//#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrBEMBody.h"
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrRadiationModel.h"
#include "FrRadiationModelBase.h"

//##CC
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
//##

namespace frydom {

  namespace internal {

    FrVariablesBEMBodyBase::FrVariablesBEMBodyBase(FrRadiationModelBase *radiationModelBase,
                                                   FrBEMBody *BEMBody,
                                                   chrono::ChVariablesBodyOwnMass *variables)
        : chrono::ChVariablesBody(*variables), m_radiationModelBase(radiationModelBase), m_BEMBody(BEMBody) {
      m_variablesBodyOwnMass = variables;
    }

    void FrVariablesBEMBodyBase::SetBEMBody(FrBEMBody *BEMBody) {
      m_BEMBody = BEMBody;
    }

    void FrVariablesBEMBodyBase::SetRadiationModelBase(FrRadiationModelBase *radiationModelBase) {
      m_radiationModelBase = radiationModelBase;
    }

    void FrVariablesBEMBodyBase::SetVariablesBodyOwnMass(chrono::ChVariablesBodyOwnMass *variables) {
      m_variablesBodyOwnMass = variables;
    }

    void FrVariablesBEMBodyBase::Compute_invMb_v(chrono::ChMatrix<double> &result,
                                                 const chrono::ChMatrix<double> &vect) const {

      result.Reset();

      auto HDB = m_radiationModelBase->GetRadiationModel()->GetHydroDB();

      if (vect.Equals(GetVariablesFb(HDB->GetBody(m_BEMBody)))) {

        for (auto bodyMotion = HDB->begin(); bodyMotion != HDB->end(); bodyMotion++) {

          if (bodyMotion->second->IsActive()) {

            auto fb = GetVariablesFb(bodyMotion->second);
            SetInBody(fb);

            auto invGeneralizedMass = m_radiationModelBase->GetInverseGeneralizedMass(m_BEMBody, bodyMotion->first);

            Vector6d result_loc;
            for (int i = 0; i < 6; i++) {
              for (int j = 0; j < 6; j++) {
                result_loc(i) += invGeneralizedMass(i, j) * fb(j);
              }
            }

            UpdateResult(result, result_loc);

          }
        }

      } else {

        auto v_loc = vect;
        SetInBody(v_loc);

        auto invGeneralizedMass = m_radiationModelBase->GetInverseGeneralizedMass(m_BEMBody, m_BEMBody);

        Vector6d result_loc;
        for (int i = 0; i < 6; i++) {
          for (int j = 0; j < 6; j++) {
            result_loc(i) += invGeneralizedMass(i, j) * v_loc(j);
          }
        }

        UpdateResult(result, result_loc);

      }

    }

    void FrVariablesBEMBodyBase::Compute_inc_invMb_v(chrono::ChMatrix<double> &result,
                                                     const chrono::ChMatrix<double> &vect) const {

      auto HDB = m_radiationModelBase->GetRadiationModel()->GetHydroDB();

      if (vect.Equals(GetVariablesFb(HDB->GetBody(m_BEMBody)))) {

        for (auto bodyMotion = HDB->begin(); bodyMotion != HDB->end(); bodyMotion++) {

          if (bodyMotion->second->IsActive()) {

            auto fb = GetVariablesFb(bodyMotion->second);
            SetInBody(fb);

            auto invGeneralizedMass = m_radiationModelBase->GetInverseGeneralizedMass(m_BEMBody, bodyMotion->first);

            Vector6d result_loc;
            for (int i = 0; i < 6; i++) {
              for (int j = 0; j < 6; j++) {
                result_loc(i) += invGeneralizedMass(i, j) * fb(j);
              }
            }

            UpdateResult(result, result_loc);

          }
        }

      } else {

        auto v_loc = vect;
        SetInBody(v_loc);

        auto invGeneralizedMass = m_radiationModelBase->GetInverseGeneralizedMass(m_BEMBody, m_BEMBody);

        Vector6d result_loc;
        for (int i = 0; i < 6; i++) {
          for (int j = 0; j < 6; j++) {
            result_loc(i) += invGeneralizedMass(i, j) * v_loc(j);
          }
        }

        UpdateResult(result, result_loc);

      }
    }

    void FrVariablesBEMBodyBase::Compute_inc_invMb_v(chrono::ChMatrix<double> &result,
                                                     const chrono::ChMatrix<double> &vect,
                                                     chrono::ChVariables *variable) const {

      auto BEMBody2 = dynamic_cast<FrVariablesBEMBodyBase *>(variable)->m_BEMBody;
      auto invGeneralizedMass = m_radiationModelBase->GetInverseGeneralizedMass(m_BEMBody, BEMBody2);

      auto v_loc = vect;
      SetInBody(v_loc);

      Vector6d result_loc;
      for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
          result_loc(i) += invGeneralizedMass(i, j) * v_loc(j);
        }
      }

      UpdateResult(result, result_loc);

    }

    void FrVariablesBEMBodyBase::Compute_inc_Mb_v(chrono::ChMatrix<double> &result,
                                                  const chrono::ChMatrix<double> &vect) const {

      auto HDB = m_radiationModelBase->GetRadiationModel()->GetHydroDB();

      if (vect.Equals(GetVariablesQb(HDB->GetBody(m_BEMBody)))) {

        for (auto bodyMotion = HDB->begin(); bodyMotion != HDB->end(); bodyMotion++) {
          if (bodyMotion->second->IsActive()) {

            auto qb = GetVariablesQb(bodyMotion->second);
            SetInBody(qb);

            auto generalizedMass = m_radiationModelBase->GetGeneralizedMass(m_BEMBody, bodyMotion->first);

            Vector6d result_loc;
            for (int i = 0; i < 6; i++) {
              for (int j = 0; j < 6; j++) {
                result_loc(i) += generalizedMass(i, j) * qb(j);
              }
            }

            UpdateResult(result, result_loc);

          }
        }

      } else {

        auto generalizedMass = m_radiationModelBase->GetGeneralizedMass(m_BEMBody, m_BEMBody);

        auto v_loc = vect;
        SetInBody(v_loc);

        Vector6d result_loc;
        for (int i = 0; i < 6; i++) {
          for (int j = 0; j < 6; j++) {
            result_loc(i) += generalizedMass(i, j) * v_loc(j);
          }
        }

        UpdateResult(result, result_loc);

      }
    }

    void FrVariablesBEMBodyBase::MultiplyAndAdd(chrono::ChMatrix<double> &result,
                                                const chrono::ChMatrix<double> &vect, const double c_a) const {

      auto v_loc = vect;
      SetInBody(v_loc);

      auto generalizedMass = m_radiationModelBase->GetGeneralizedMass(m_BEMBody, m_BEMBody);

      Vector6d result_loc;
      for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
          result_loc[i] += c_a * generalizedMass(i, j) * v_loc(j);
        }
      }

      UpdateResult(result, result_loc, this->offset);

    }

    void FrVariablesBEMBodyBase::DiagonalAdd(chrono::ChMatrix<double> &result, const double c_a) const {

      auto generalizedMass = m_radiationModelBase->GetGeneralizedMass(m_BEMBody, m_BEMBody);

      for (int i = 0; i < 6; i++) {
        result(this->offset + i) += c_a * generalizedMass(i, i);
      }

    }

    void FrVariablesBEMBodyBase::Build_M(chrono::ChSparseMatrix &storage, int insrow, int inscol, const double c_a) {

      auto generalizedMass = m_radiationModelBase->GetGeneralizedMass(m_BEMBody, m_BEMBody);

      for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
          storage.SetElement(insrow + i, inscol + j, c_a * generalizedMass(i, j));
        }
      }
    }

    chrono::ChMatrix<double> FrVariablesBEMBodyBase::GetVariablesFb(frydom::FrBody *body) const {
      return internal::GetChronoBody(body)->Variables().Get_fb();
    }

    chrono::ChMatrix<double> FrVariablesBEMBodyBase::GetVariablesQb(frydom::FrBody *body) const {
      return internal::GetChronoBody(body)->Variables().Get_qb();
    }


    // protected

    void FrVariablesBEMBodyBase::UpdateResult(chrono::ChMatrix<double>& result, mathutils::Vector6d<double> &vect, int offset) const {

      auto eq_frame = m_radiationModelBase->GetRadiationModel()->GetHydroDB()->GetMapper()->GetEquilibriumFrame(m_BEMBody);
      auto vect_world = eq_frame->GetFrame().ProjectVectorFrameInParent(Position(vect(0), vect(1) ,vect(2)), NWU);
      for (int i=0; i<3; i++) result(offset + i) += vect_world[i];
      for (int i=3; i<6; i++) result(offset + i) += vect[i];

    }

    void FrVariablesBEMBodyBase::SetInBody(chrono::ChMatrix<double> &vect) const {

      auto eq_frame = m_radiationModelBase->GetRadiationModel()->GetHydroDB()->GetMapper()->GetEquilibriumFrame(m_BEMBody);

      auto v_loc = Position(vect(0), vect(1), vect(2));
      auto vect_loc = eq_frame->GetFrame().ProjectVectorParentInFrame(v_loc, NWU);
      for (int i =0; i<3; i++) vect(i) = vect_loc[i];

    }

  } // end namespace internal

} // end namespace frydom
