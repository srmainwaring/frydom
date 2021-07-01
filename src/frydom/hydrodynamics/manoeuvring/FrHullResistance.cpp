//
// Created by lletourn on 01/07/2021.
//

#include "FrHullResistance.h"

namespace frydom {


  FrQuadHullResistance::FrQuadHullResistance(double a_pos, double a_neg, double b_pos, double b_neg):
  m_a_pos(a_pos), m_a_neg(a_neg), m_b_pos(b_pos), m_b_neg(b_neg){

  }

  double FrQuadHullResistance::Rh(double u) const {
    double rh;
    if (u>=0) {
      rh = m_a_pos * u * u + m_b_pos * u;
    } else {
      rh = m_a_neg * u * u + m_b_neg * u;
    }
    return rh;
  }

  FrInterpHullResistance::FrInterpHullResistance(const std::shared_ptr<std::vector<double>> &u,
                                                 const std::shared_ptr<std::vector<double>> &Rh) {
    m_data.Initialize(u, Rh);
  }

  double FrInterpHullResistance::Rh(double u) const {
    return m_data.Eval(u);
  }

  double FrInterpHullResistance::GetUMin() const {
    return m_data.GetXmin();
  }

  double FrInterpHullResistance::GetUMax() const {
    return m_data.GetXmax();
  }
}