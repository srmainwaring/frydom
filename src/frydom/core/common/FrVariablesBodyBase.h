//
// Created by camille on 09/04/2020.
//

#ifndef FRYDOM_FRVARIABLESBODYBASE_H
#define FRYDOM_FRVARIABLESBODYBASE_H

#include "chrono/solver/ChVariablesBody.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"
#include "chrono/core/ChMatrix.h"

#include "MathUtils/Matrix66.h"

namespace frydom {

  // foward declaration
  class FrBody;

 namespace internal {

  class FrVariablesBodyBase : public chrono::ChVariablesBody {

   protected:

    chrono::ChVariablesBodyOwnMass* m_variablesBodyOwnMass;
    mathutils::Matrix66<double> m_mass;
    mathutils::Matrix66<double> m_inv_mass;

   public:

    // Constructor

    FrVariablesBodyBase(chrono::ChVariablesBodyOwnMass* variables);

    // Compute

    void Compute_invMb_v(chrono::ChVectorRef result, chrono::ChVectorConstRef vect) const override;

    void Compute_inc_invMb_v(chrono::ChVectorRef result, chrono::ChVectorConstRef vect) const override;

    void Compute_inc_Mb_v(chrono::ChVectorRef result, chrono::ChVectorConstRef vect) const override;

    void MultiplyAndAdd(chrono::ChVectorRef result, chrono::ChVectorConstRef vect,
                        const double c_a) const override;

    void DiagonalAdd(chrono::ChVectorRef result, const double c_a) const override;

    void Build_M(chrono::ChSparseMatrix& storage, int insrow, int inscol, const double c_a) override;

    // Mass

    double GetBodyMass() const override;

    chrono::ChMatrix33<>& GetBodyInertia() override;

    const chrono::ChMatrix33<>& GetBodyInertia() const override;

    chrono::ChMatrix33<>& GetBodyInvInertia() override;

    const chrono::ChMatrix33<>& GetBodyInvInertia() const override;

  };

 } // end namespace internal
} // end namespace frydom

#endif //FRYDOM_FRVARIABLESBODYBASE_H
