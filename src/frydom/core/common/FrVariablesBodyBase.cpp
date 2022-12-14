//
// Created by camille on 09/04/2020.
//

#include <frydom/core/body/FrBody.h>
#include "FrVariablesBodyBase.h"
#include "MathUtils/Vector6d.h"
#include "frydom/core/math/FrMatrix.h"

namespace frydom {

  namespace internal {

    FrVariablesBodyBase::FrVariablesBodyBase(chrono::ChVariablesBodyOwnMass* variables)
        : chrono::ChVariablesBody(*variables) {

      m_variablesBodyOwnMass = variables;

      // Set mass matrix 6x6
      m_mass.setZero();
      m_mass(0, 0) = m_variablesBodyOwnMass->GetBodyMass();
      m_mass(1, 1) = m_variablesBodyOwnMass->GetBodyMass();
      m_mass(2, 2) = m_variablesBodyOwnMass->GetBodyMass();
      m_mass.block<3, 3>(3, 3) = m_variablesBodyOwnMass->GetBodyInertia();

      // Compute the inverse mass matrix
      m_inv_mass = m_mass.inverse();

      // Replace chrono variables with this new variables
      //internal::GetChronoBody(body)->SetVariables(std::shared_ptr<FrVariablesBodyBase>(this));
    }

    void FrVariablesBodyBase::Compute_invMb_v(chrono::ChVectorRef result,
                                              chrono::ChVectorConstRef vect) const {
      assert(vect.size() == Get_ndof());
      assert(result.size() == Get_ndof());

      result.setZero();
      for (unsigned int i = 0; i < 6; i++) {
        for (unsigned int j = 0; j < 6; j++) {
          result(i) += m_inv_mass(i, j) * vect(j);
        }
      }
    }

    void FrVariablesBodyBase::Compute_inc_invMb_v(chrono::ChVectorRef result,
                                                  chrono::ChVectorConstRef vect) const {
      assert(vect.size() == Get_ndof());
      assert(result.size() == Get_ndof());

      for (unsigned int i = 0; i < 6; i++) {
        for (unsigned int j = 0; j < 6; j++) {
          result(i) += m_inv_mass(i, j) * vect(j);
        }
      }
    }

    void FrVariablesBodyBase::Compute_inc_Mb_v(chrono::ChVectorRef result,
                                               chrono::ChVectorConstRef vect) const {
      assert(vect.size() == Get_ndof());
      assert(result.size() == Get_ndof());

      for (unsigned int i = 0; i < 6; i++) {
        for (unsigned int j = 0; j < 6; j++) {
          result(i) += m_mass(i, j) * vect(j);
        }
      }
    }

    void FrVariablesBodyBase::MultiplyAndAdd(chrono::ChVectorRef result,
                                             chrono::ChVectorConstRef vect,
                                             const double c_a) const {
      assert(result.size() == 1 && vect.size() == 1);

      mathutils::Vector6d<double> q;
      for (unsigned int i = 0; i < 6; ++i) {
        q[i] = vect(this->offset + i);
      }

      mathutils::Matrix66<double> scaledmass = c_a * m_mass;

      for (unsigned int i = 0; i < 6; ++i) {
        result(this->offset + i) = scaledmass.GetRow(i) * q;
      }
    }

    void FrVariablesBodyBase::DiagonalAdd(chrono::ChVectorRef result, const double c_a) const {
      assert(result.size() == 1);
      for (unsigned int i = 0; i < 6; ++i) {
        result(this->offset + i) += c_a * m_mass(i, i);
      }
    }

    void FrVariablesBodyBase::Build_M(chrono::ChSparseMatrix &storage, int insrow, int inscol, const double c_a) {
      // TODO
    }

    // Mass

    double FrVariablesBodyBase::GetBodyMass() const {
      return m_variablesBodyOwnMass->GetBodyMass();
    }

    chrono::ChMatrix33<> &FrVariablesBodyBase::GetBodyInertia() {
      return m_variablesBodyOwnMass->GetBodyInertia();
    }

    const chrono::ChMatrix33<> &FrVariablesBodyBase::GetBodyInertia() const {
      return m_variablesBodyOwnMass->GetBodyInertia();
    }

    chrono::ChMatrix33<> &FrVariablesBodyBase::GetBodyInvInertia() {
      return m_variablesBodyOwnMass->GetBodyInvInertia();
    }

    const chrono::ChMatrix33<> &FrVariablesBodyBase::GetBodyInvInertia() const {
      return m_variablesBodyOwnMass->GetBodyInvInertia();
    }

  } // end namespace internal
} // end namespace frydom
