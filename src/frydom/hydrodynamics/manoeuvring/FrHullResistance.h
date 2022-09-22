// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ========================================================================

#ifndef FRYDOM_FRHULLRESISTANCE_H
#define FRYDOM_FRHULLRESISTANCE_H

#include "MathUtils/Interp1d.h"
#include "frydom/core/force/FrForce.h"
#include "frydom/io/JSONNode.h"

namespace frydom {

  // -------------------------------------------------------------------
  // Base hull resistance model
  // -------------------------------------------------------------------

  class FrHullResistance {

   public:

    virtual double Rh(double u) const = 0;

    virtual double GetUMin() const = 0;

    virtual double GetUMax() const = 0;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  // --------------------------------------------------------------------
  // Table hull resistance model
  // --------------------------------------------------------------------

  class FrInterpHullResistance : public FrHullResistance {
   public:

    FrInterpHullResistance(const JSONNode& node);

    FrInterpHullResistance(const std::shared_ptr<std::vector<double>>& u, const std::shared_ptr<std::vector<double>>& Rh);

    double Rh(double u) const override;

    double GetUMin() const override;

    double GetUMax() const override;

   private:
    mathutils::Interp1dLinear<double, double> m_data;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  // ----------------------------------------------------------------
  // Quadratic hull resistance model
  // ----------------------------------------------------------------

  class FrQuadHullResistance : public FrHullResistance {

   public:

    FrQuadHullResistance(const JSONNode& node);

    FrQuadHullResistance(double a_pos, double a_neg, double b_pos, double b_neg);

    double Rh(double u) const override;

    double GetUMin() const override;

    double GetUMax() const override;

   private:

    double m_a_pos, m_a_neg, m_b_pos, m_b_neg;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  // --------------------------------------------------------------------------
  // FORCE
  // --------------------------------------------------------------------------

  class FrHullResistanceForce : public FrForce {

    public:
      explicit FrHullResistanceForce(const std::string& name, FrBody* body,
                                     const JSONNode& node, const Position& mid_ship);

      void Initialize() override;

    private:

      void Compute(double time) override;

    private:
      std::shared_ptr<FrHullResistance> m_hullResistance;
      Position m_mid_ship;

  };

  // ---------------------------------------------------------------------------
  // MAKERS
  // ---------------------------------------------------------------------------

  std::shared_ptr<FrHullResistance> make_hull_resistance(const JSONNode& node);

  std::shared_ptr<FrHullResistanceForce> make_hull_resistance_force(const std::string& name, std::shared_ptr<FrBody> body,
                                                                    const JSONNode& node, const Position& mid_ship);

} // end namespace frydom
#endif //FRYDOM_FRHULLRESISTANCE_H
