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

#ifndef FRYDOM_FRMORISONADDEDMASS_H
#define FRYDOM_FRMORISONADDEDMASS_H

#include "chrono/fea/ChElementGeneric.h"
#include "MathUtils/MathUtils.h"

namespace frydom {

  // Forward declaration
  class FrMorisonCompositeElement;

  namespace internal {

    class FrMorisonAddedMassBase : public chrono::fea::ChElementGeneric {

      public:

        explicit FrMorisonAddedMassBase(FrMorisonCompositeElement* morisonModel);

        int GetNnodes() override;
        int GetNdofs() override;
        int GetNodeNdofs(int n) override;
        std::shared_ptr<chrono::fea::ChNodeFEAbase> GetNodeN(int n) override;

        void GetStateBlock(chrono::ChVectorDynamic<>& mD) override;

        void ComputeKRMmatricesGlobal(chrono::ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) override;

        void ComputeInternalForces(chrono::ChVectorDynamic<>& Fi) override;

        void ComputeGravityForces(chrono::ChVectorDynamic<>& Fi, const chrono::ChVector<>& G_acc) override;

        void EleIntLoadResidual_F(chrono::ChVectorDynamic<>& R, const double c) override;

        void EleIntLoadResidual_Mv(chrono::ChVectorDynamic<>& R, const chrono::ChVectorDynamic<>& w, const double c) override;

        void EleIntLoadResidual_F_gravity(chrono::ChVectorDynamic<>& R, const chrono::ChVector<>& G_acc, const double c) override;

        void Initialize();

        void Update();

      protected:

        void SetVar();

      private:

        void SetupInitial(chrono::ChSystem* system) override;

        FrMorisonCompositeElement* m_frydomMorisonModel;

        mathutils::MatrixMN<double> m_addedMassMatrix;
    };

  } // end namespace internal
} // end namespace frydom

#endif //FRYDOM_FRMORISONADDEDMASS_H
