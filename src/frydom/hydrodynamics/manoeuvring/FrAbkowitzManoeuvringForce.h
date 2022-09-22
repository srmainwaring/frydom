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

#ifndef FRYDOM_FRABKOWITZMANOEUVRINGFORCE_H
#define FRYDOM_FRABKOWITZMANOEUVRINGFORCE_H

#include "frydom/core/force/FrForce.h"
#include "frydom/io/JSONNode.h"

namespace frydom {

  // Forward declaration
  class FrHullResistance;

  /// Abkowitz manoeuvring model from Yoshimura (2012)
  class FrAbkowitzManoeuvringForce : public FrForce {

   public:

    /// Constructor of Abkowitz manoeuvring model from json file (for backward compatibility)
    /// DEPRECIATED : prefer to use constructor from JSON node directly
    /// \param name Manoeuvring model name
    /// \param body Body ship to which the manoeuvring model is applied
    /// \param file JSON file path with manoeuvring model definitions
    /// \param mid_ship Mid ship position in body reference frame
    FrAbkowitzManoeuvringForce(const std::string &name, FrBody *body,
                               const std::string &file,
                               const Position &mid_ship);

    /// Constructor of Abkowitz manoeuvring model from JSON node
    /// \param name Manoeuvring model name
    /// \param body Body ship to which the manoeuvring model is applied
    /// \param node_man JSON node with hydrodynamic derivative definitions
    /// \param node_res JSON node with calm water resistance model parameters
    /// \param mid_ship Mid ship position in body reference frame
    /// \param draft_m Ship draft
    /// \param lpp_m Ship length between perpendicular
    FrAbkowitzManoeuvringForce(const std::string &name, FrBody *body,
                               const JSONNode &node_man, const JSONNode &node_res,
                               const Position &mid_ship, double draft_m, double lpp_m);

    /// Call force initialization
    void Initialize() override;

    /// Get min ship speed from resistance curve (in m/s)
    double GetUMin() const;

    /// Get max ship speed from resistance curve (in m/s)
    double GetUMax() const;

   private:

    void Compute(double time) override;

    void DefineLogMessages() override;

    void LoadResistanceCurve(const std::string& filepath);

    void LoadAbkowitzManoeuvringFile(const std::string& filepath);

   private:

    std::shared_ptr<FrHullResistance> m_hullResistance;

    double m_Lpp;
    double m_draft;
    Position m_mid_ship;

    double m_Xvv, m_Xvvvv, m_Xrr, m_Xvr;
    double m_Yv, m_Yvvv, m_Yvrr, m_Yr, m_Yrrr, m_Yvvr;
    double m_Nv, m_Nvvv, m_Nvrr, m_Nr, m_Nrrr, m_Nvvr;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  // MAKERS

  std::shared_ptr<FrAbkowitzManoeuvringForce>
      make_abkowitz_manoeuvring_model(const std::string& name, std::shared_ptr<FrBody> body,
                                      const std::string& file, const Position& mid_ship);

  std::shared_ptr<FrAbkowitzManoeuvringForce>
      make_abkowitz_manoeuvring_model(const std::string& name, std::shared_ptr<FrBody> body,
                                      const JSONNode& node_man, const JSONNode& node_res,
                                      const Position& mid_ship,double draft_m, double lpp_m);

}  // end namespace frydom

#endif //FRYDOM_FRABKOWITZMANOEUVRINGFORCE_H
