//
// Created by camille on 15/05/19.
//

#ifndef FRYDOM_FRVARIABLESBEMBODYBASE_H
#define FRYDOM_FRVARIABLESBEMBODYBASE_H

#include "chrono/solver/ChVariablesBody.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"

#include "MathUtils/Matrix66.h"
#include "MathUtils/Vector6d.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrBEMBody.h"

namespace frydom {

  // Forward declaration
  class FrBody;

//  class FrBEMBody;

  namespace internal {

    // Forward declaration
    class FrRadiationModelBaseVariable;

    class FrVariablesBEMBodyBase : public chrono::ChVariablesBody {

     private:

      FrRadiationModelBaseVariable *m_radiationModelBase;
      FrBEMBody *m_BEMBody;
      chrono::ChVariablesBodyOwnMass *m_variablesBodyOwnMass;

     public:

      FrVariablesBEMBodyBase() : ChVariablesBody() {}

      explicit FrVariablesBEMBodyBase(FrRadiationModelBaseVariable *radiationModelBase,
                                      FrBEMBody *BEMBody,
                                      chrono::ChVariablesBodyOwnMass *variables);

      void SetBEMBody(FrBEMBody *BEMBody);

      FrBEMBody *GetBEMBody() const { return m_BEMBody; }

      void SetRadiationModelBase(FrRadiationModelBaseVariable *radiationModelBase);

      void SetVariablesBodyOwnMass(chrono::ChVariablesBodyOwnMass *variables);

      void Compute_invMb_v(chrono::ChVectorRef result, chrono::ChVectorConstRef vect) const override;

      void Compute_inc_invMb_v(chrono::ChVectorRef result, chrono::ChVectorConstRef vect) const override;

      void Compute_inc_invMb_v(chrono::ChVectorRef result, chrono::ChVectorConstRef vect,
                               chrono::ChVariables *variable) const;

      void Compute_inc_Mb_v(chrono::ChVectorRef result, chrono::ChVectorConstRef vect) const override;

      void MultiplyAndAdd(chrono::ChVectorRef result, chrono::ChVectorConstRef vect,
                          const double c_a) const override;

      void DiagonalAdd(chrono::ChVectorRef result, const double c_a) const override;

      void Build_M(chrono::ChSparseMatrix &storage, int insrow, int inscol, const double c_a) override;

      chrono::ChVectorRef GetVariablesFb(FrBody *body) const;

      chrono::ChVectorRef GetVariablesQb(FrBody *body) const;

      //
      // VIRTUAL FUNCTION
      //


      double GetBodyMass() const override { return m_variablesBodyOwnMass->GetBodyMass(); }

      chrono::ChMatrix33<> &GetBodyInertia() override { return m_variablesBodyOwnMass->GetBodyInertia(); }

      const chrono::ChMatrix33<> &GetBodyInertia() const override { return m_variablesBodyOwnMass->GetBodyInertia(); }

      chrono::ChMatrix33<> &GetBodyInvInertia() override { return m_variablesBodyOwnMass->GetBodyInvInertia(); }

      const chrono::ChMatrix33<> &
      GetBodyInvInertia() const override { return m_variablesBodyOwnMass->GetBodyInvInertia(); }

     protected:

      void UpdateResult(chrono::ChVectorRef result, mathutils::Vector6d<double> &vect, int offset=0) const;

      void SetInBody(chrono::ChVectorN<double, 6> vect) const;

     public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };

  } // end namespace internal

} // end namespace frydom

#endif //FRYDOM_FRVARIABLESBEMBODYBASE_H
