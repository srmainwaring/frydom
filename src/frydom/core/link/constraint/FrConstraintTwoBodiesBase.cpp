//
// Created by camille on 18/06/19.
//

#include "FrConstraintTwoBodiesBase.h"
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrVariablesBEMBodyBase.h"

namespace frydom {

  namespace internal {

    void FrConstraintTwoBodiesBase::Update_auxiliary() {

      // 1- Assuming jacobians are already computed, now compute
      //   the matrices [Eq_a]=[invM_a]*[Cq_a]' and [Eq_b]
      if (variables_a->IsActive()) {
        variables_a->Compute_invMb_v(Eq_a, Cq_a.transpose());
      }
      if (variables_b->IsActive()) {
        variables_b->Compute_invMb_v(Eq_b, Cq_b.transpose());
      }

      // Off-diagonal mass-matrix coefficients
      if (variables_a->IsActive() and variables_b->IsActive()) {

        auto varBEM_a = dynamic_cast<FrVariablesBEMBodyBase *>(variables_a);
        auto varBEM_b = dynamic_cast<FrVariablesBEMBodyBase *>(variables_b);

        if (varBEM_a and varBEM_b) {
          varBEM_a->Compute_inc_invMb_v(Eq_a, Cq_b.transpose(), variables_b);
          varBEM_b->Compute_inc_invMb_v(Eq_b, Cq_a.transpose(), variables_a);
        }
      }

      // 2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
      chrono::ChMatrixNM<double, 1, 1> res;
      g_i = 0;
      if (variables_a->IsActive()) {
        g_i += Cq_a * Eq_a;
      }
      if (variables_b->IsActive()) {
        g_i += Cq_b * Eq_b;
      }

      // 3- adds the constraint force mixing term (usually zero):
      if (cfm_i)
        g_i += cfm_i;
    }

  } // end namespace internal
} // end namespace frydom
