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
  class FrHullResistanceForce;

  struct FrHydroDerivatives {
    double m_Xvv = 0;
    double m_Xvvvv = 0;
    double m_Xvr = 0;
    double m_Xrr = 0;
    double m_Yv = 0;
    double m_Yr = 0;
    double m_Yvvv = 0;
    double m_Yvvr = 0;
    double m_Yvrr = 0;
    double m_Yrrr = 0;
    double m_Nv = 0;
    double m_Nr = 0;
    double m_Nvvv = 0;
    double m_Nvvr = 0;
    double m_Nvrr = 0;
    double m_Nrrr = 0;
  };

  /// Abkowitz manoeuvring model from Yoshimura (2012)
  class FrAbkowitzManoeuvringForce : public FrForce {

   public:


    FrAbkowitzManoeuvringForce(const std::string& name, FrBody* body,
                               const FrHydroDerivatives& hydroDeriv,
                               const std::shared_ptr<FrHullResistanceForce>& hullResistanceForce,
                               const Position& mid_ship, double draft_m, double lpp_m);

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

    std::shared_ptr<FrHullResistanceForce> m_hullResistanceForce;

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
                                  const FrHydroDerivatives& hydroDerive,
                                  const std::shared_ptr<FrHullResistanceForce>& hullResistanceForce,
                                  const Position& mid_ship, double draft_m, double lpp_m);


  }  // end namespace frydom

#endif //FRYDOM_FRABKOWITZMANOEUVRINGFORCE_H
