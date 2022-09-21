//
// Created by lletourn on 01/07/2021.
//

#ifndef FRYDOM_FRHULLRESISTANCE_H
#define FRYDOM_FRHULLRESISTANCE_H

#include "MathUtils/Interp1d.h"

namespace frydom {

  class FrHullResistance {

   public:

    virtual double Rh(double u) const = 0;

    virtual double GetUMin() const = 0;

    virtual double GetUMax() const = 0;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  class FrInterpHullResistance : public FrHullResistance {
   public:
    FrInterpHullResistance(const std::shared_ptr<std::vector<double>>& u, const std::shared_ptr<std::vector<double>>& Rh);

    double Rh(double u) const override;

    double GetUMin() const override;

    double GetUMax() const override;

   private:
    mathutils::Interp1dLinear<double, double> m_data;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  class FrQuadHullResistance : public FrHullResistance {

   public:

    FrQuadHullResistance(double a_pos, double a_neg, double b_pos, double b_neg);

    double Rh(double u) const override;

    double GetUMin() const override;

    double GetUMax() const override;

   private:

    double m_a_pos, m_a_neg, m_b_pos, m_b_neg;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

} // end namespace frydom
#endif //FRYDOM_FRHULLRESISTANCE_H
