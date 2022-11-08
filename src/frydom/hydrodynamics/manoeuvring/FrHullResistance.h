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

namespace frydom {

  // -------------------------------------------------------------------
  // Base hull resistance model
  // -------------------------------------------------------------------

  class FrHullResistanceForce: public FrForce {

   public:

    explicit FrHullResistanceForce(const std::string& name, FrBody* body, const Position& mid_ship);

    void Initialize() override;

    virtual double Rh(double u) const = 0;

    virtual double GetUMin() const = 0;

    virtual double GetUMax() const = 0;

   private:

    void Compute(double time) override;

   protected:
    Position m_mid_ship;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  // --------------------------------------------------------------------
  // Table hull resistance model
  // --------------------------------------------------------------------

  class FrInterpHullResistanceForce : public FrHullResistanceForce {
   public:
    explicit FrInterpHullResistanceForce(const std::string& name, FrBody* body, const Position& mid_ship,
                                         std::shared_ptr<std::vector<double>> u,
                                         std::shared_ptr<std::vector<double>> Rh);

    double Rh(double u) const override;

    double GetUMin() const override;

    double GetUMax() const override;

   private:
    mathutils::Interp1dLinear<double, double> m_data;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  std::shared_ptr<FrInterpHullResistanceForce> make_interp_hull_resistance(
      const std::string& name, FrBody* body, const Position& mid_ship,
      std::shared_ptr<std::vector<double>> u,
      std::shared_ptr<std::vector<double>> Rh);

  // ----------------------------------------------------------------
  // Quadratic hull resistance model
  // ----------------------------------------------------------------

  class FrQuadHullResistanceForce : public FrHullResistanceForce {

   public:
    explicit FrQuadHullResistanceForce(const std::string& name, FrBody* body, const Position& mid_ship,
                                       double a_pos, double a_neg, double b_pos, double b_neg,
                                       double lpp_m);

    double Rh(double u) const override;

    double GetUMin() const override;

    double GetUMax() const override;

   private:
    double m_a_pos, m_a_neg, m_b_pos, m_b_neg;
    double m_lpp;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  std::shared_ptr<FrQuadHullResistanceForce> make_quadratic_hull_resistance(
      const std::string& name, FrBody* body, const Position& mid_ship,
      double a_pos, double a_neg, double b_pos, double b_neg, double lpp_m);

} // end namespace frydom
#endif //FRYDOM_FRHULLRESISTANCE_H
