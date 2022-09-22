//
// Created by lletourn on 01/07/2021.
//

#ifndef FRYDOM_FRHULLRESISTANCE_H
#define FRYDOM_FRHULLRESISTANCE_H

#include "MathUtils/Interp1d.h"

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

  // ---------------------------------------------------------------------------
  // MAKERS
  // ---------------------------------------------------------------------------

  std::shared_ptr<FrHullResistance> make_hull_resistance(const JSONNode& node);

} // end namespace frydom
#endif //FRYDOM_FRHULLRESISTANCE_H
