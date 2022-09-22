//
// Created by lletourn on 01/07/2021.
//

#include "FrHullResistance.h"
#include "MathUtils/Unit.h"

#define VESSEL_WORLD_RECORD_SPEED_KT 80

namespace frydom {

  // --------------------------------------------------------------------------
  // Quadratic hull resistance
  // --------------------------------------------------------------------------

  FrQuadHullResistance::FrQuadHullResistance(const JSONNode& node) :
    FrQuadHullResistance(node.get<double>("a+"), node.get<double>("a-"),
        node.get<double>("b+"), node.get<double>("b-")) {}

  FrQuadHullResistance::FrQuadHullResistance(double a_pos, double a_neg, double b_pos, double b_neg):
  m_a_pos(a_pos), m_a_neg(a_neg), m_b_pos(b_pos), m_b_neg(b_neg) {}

  double FrQuadHullResistance::Rh(double u) const {
    double rh;
    if (u>=0) {
      rh = m_a_pos * u * u + m_b_pos * u;
    } else {
      rh = m_a_neg * u * u + m_b_neg * u;
    }
    return rh;
  }

  double FrQuadHullResistance::GetUMin() const {
    return - 80 * MU_KNOT;
  }

  double FrQuadHullResistance::GetUMax() const {
    return 80 * MU_KNOT;
  }


  // -----------------------------------------------------------------------------
  // Interp hull resistance
  // -----------------------------------------------------------------------------

  FrInterpHullResistance::FrInterpHullResistance(const JSONNode& node) {
    auto u = std::make_shared<std::vector<double>>(node.get<std::vector<double>>("velocity_kt"));
    for (auto &vel : *u) vel = convert_velocity_unit(vel, mathutils::KNOT, mathutils::MS);
    auto Rh = std::make_shared<std::vector<double>>(node.get<std::vector<double>>("resistance_N"));
    m_data.Initialize(u, Rh);

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

  std::shared_ptr<FrHullResistance> make_hull_resistance(const JSONNode& node) {
    if (node.exists("lut")) {
      return std::make_shared<FrInterpHullResistance>(node["lut"]);
    } else if (node.exists("quadratic")) {
      return std::make_shared<FrQuadHullResistance>(node["quadratic"]);
    } else {
      std::cerr << "warning : unknown hull resistance model " << std::endl;
      exit(EXIT_FAILURE);
    }
  }

}