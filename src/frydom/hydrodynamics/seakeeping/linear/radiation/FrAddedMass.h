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

#ifndef FRYDOM_FRADDEDMASS_H
#define FRYDOM_FRADDEDMASS_H

#include "chrono/fea/ChElementGeneric.h"
#include "MathUtils/MathUtils.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrBEMBody.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "frydom/cable/fea/FrFEANode.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/cable/fea/FrFEALink.h"

namespace frydom {

  // Forward declaration
  class FrRadiationModel;
  class FrHydroDB;


  // hash defition for the map with pair as a key
  struct pair_hash {
    template<class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &p) const {
      auto h1 = std::hash<T1>{}(p.first);
      auto h2 = std::hash<T2>{}(p.second);

      // Mainly for demonstration purposes, i.e. works but is overly simple
      // In the real world, use sth. like boost.hash_combine
      return h1 ^ h2;
    }
  };

  // ------------------------------------------------------------------------------
  // FrAddedMassBase
  // ------------------------------------------------------------------------------

  namespace internal {

    class FrAddedMassBase : public chrono::fea::ChElementGeneric {

      public:

        explicit FrAddedMassBase(FrRadiationModel* radiationModel);
        ~FrAddedMassBase();

        FrRadiationModel* GetRadiationModel() const { return m_frydomRadiationModel; }

        int GetNnodes() override;
        int GetNdofs() override;
        int GetNodeNdofs(int n) override;
        std::shared_ptr<chrono::fea::ChNodeFEAbase> GetNodeN(int n) override;

        void GetStateBlock(chrono::ChVectorDynamic<>& mD) override;

        void ComputeKRMmatricesGlobal(chrono::ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) override;

        void ComputeInternalForces(chrono::ChVectorDynamic<>& Fi) override;

        void ComputeGravityForces(chrono::ChVectorDynamic<>& Fi, const chrono::ChVector<>& G_acc) override;

        void BuildGeneralizedMass();

        void EleIntLoadResidual_F(chrono::ChVectorDynamic<>& R, const double c) override;

        void EleIntLoadResidual_Mv(chrono::ChVectorDynamic<>& R, const chrono::ChVectorDynamic<>& w, const double c) override;

        void EleIntLoadResidual_F_gravity(chrono::ChVectorDynamic<>& R, const chrono::ChVector<>& G_acc, const double c) override;

        std::vector<std::shared_ptr<internal::FrFEALinkBase>> GetLinks() { return m_links; }

        void Initialize();

        void SetActive(bool is_active);

      protected:

        void SetNodes();

      private:

        void SetupInitial(chrono::ChSystem* system) override;

        FrRadiationModel* m_frydomRadiationModel;

        std::unordered_map<std::pair<FrBEMBody*, FrBEMBody*>, mathutils::Matrix66<double>, pair_hash> m_generalizedAddedMass;
        mathutils::MatrixMN<double> m_addedMassMatrix;
        int m_nb_bodies;

        std::vector<std::shared_ptr<internal::FrFEANodeBase>> m_nodes;
        std::vector<std::shared_ptr<internal::FrBodyBase>> m_bodies;
        std::vector<std::shared_ptr<internal::FrFEALinkBase>> m_links;

        FrHydroDB* m_hdb;

        bool m_is_active;

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };

  } // end namespace internal
} // end namespace frydom

#endif //FRYDOM_FRADDEDMASS_H
