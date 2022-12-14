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


#include "chrono/core/ChFrame.h"

#include "FrTidalModel.h"
#include "frydom/logging/FrEventLogger.h"


namespace frydom {

  void FrUTCTime::Check() {
    assert(m_hours > 0. && m_hours < 24);
    assert(m_minutes > 0. && m_minutes <= 60.);
    assert(m_seconds > 0. && m_seconds <= 60.);
    assert(m_local_correction > -24 && m_local_correction < 24);  // TODO verifier
  }

  FrUTCTime::FrUTCTime(const unsigned int hours, const unsigned int minutes, const unsigned int seconds) :
      m_hours(hours),
      m_minutes(minutes),
      m_seconds(seconds) {
    Check();
  }

  FrUTCTime::FrUTCTime(const unsigned int hours, const unsigned int minutes) :
      m_hours(hours),
      m_minutes(minutes),
      m_seconds(0) {
    Check();
  }

  double FrUTCTime::GetHours() const {
    return (double) m_hours + (double) m_minutes / 60. + (double) m_seconds / 3600.;
  }

  double FrUTCTime::GetMinutes() const {
    return (double) m_hours * 60 + (double) m_minutes + (double) m_seconds / 60.;
  }

  double FrUTCTime::GetSeconds() const {
    return (double) m_hours * 3600 + (double) m_minutes * 60 + (double) m_seconds;
  }

  void FrTidal::BuildTable() {
    switch (m_mode) {
      case NO_TIDAL:
        return;
      case TWELFTH_RULE:
        BuildTwelfthRuleTable();
    }
  }

  void FrTidal::BuildTwelfthRuleTable() {
    double h_twelfth = fabs(m_h2 - m_h1) / 12.; // One twelfth

    double t1 = m_t1.GetMinutes();
    double t2 = m_t2.GetMinutes();

    double dt = (t2 - t1) / 6.;

    std::vector<double> timeVect;
    timeVect.reserve(7);
    for (uint it = 0; it < 7; ++it) {
      timeVect.push_back(t1 + it * dt);  // TODO: verifier que la derniere valeur est egale a t2 !!
    }

    std::vector<double> hVect;
    hVect.reserve(13);

    int op;
    if (m_level1 == LOW) {
      op = 1;
    } else {
      op = -1;
    }

    hVect.push_back(m_h1);  // Last extremum tidal level
    hVect.push_back(m_h1 + op * 1 * h_twelfth); // 1 twelfth
    hVect.push_back(m_h1 + op * 3 * h_twelfth); // 2 twelfth
    hVect.push_back(m_h1 + op * 6 * h_twelfth); // 3 twelfth
    hVect.push_back(m_h1 + op * 9 * h_twelfth); // 3 twelfth
    hVect.push_back(m_h1 + op * 11 * h_twelfth); // 2 twelfth
    hVect.push_back(m_h1 + op * 12 * h_twelfth); // 1 twelfth // TODO: verifier qu'on a h2...

    // Populating the interpolation table
    m_tidal_table.SetX(timeVect);
    m_tidal_table.AddY("tidal_height", hVect);
  }

  FrTidal::FrTidal(FrFreeSurface *freeSurface) :
      m_freeSurface(freeSurface),
      m_tidal_table(mathutils::LINEAR) {
    m_tidalFrame = std::make_unique<chrono::ChFrame<double>>();
  }

  FrTidal::FrTidal(FrFreeSurface *freeSurface,
                   const FrUTCTime t1, const double h1, FrTidal::TIDAL_LEVEL level1, const FrUTCTime t2,
                   const double h2, FrTidal::TIDAL_LEVEL level2) :
      m_t1(t1),
      m_h1(h1),
      m_level1(level1),
      m_t2(t2),
      m_h2(h2),
      m_level2(level2),
      m_mode(TWELFTH_RULE),
      m_freeSurface(freeSurface),
      m_tidal_table(mathutils::LINEAR) {

    assert(h1 >= 0. && h2 >= 0.);
    assert(level1 != level2);  // Levels have to be different
    assert(t2.GetSeconds() > t1.GetSeconds());  // Ajouter operateur de comparaison de temps

    BuildTable();

  }

  void FrTidal::Update(const double time) {
    double waterHeight = 0.;

    if (m_mode == NO_TIDAL) {
      waterHeight = m_h1;
    }

    if (m_mode == TWELFTH_RULE) {
      waterHeight = m_tidal_table.Eval("tidal_height", m_time);
    }

    m_tidalFrame->GetPos().z() = waterHeight;
  }

  double FrTidal::GetHeight(FRAME_CONVENTION fc) const {
    double ZPos = m_tidalFrame->GetPos().z();
    if (IsNED(fc)) { ZPos = -ZPos; }
    return ZPos;
  }

  const chrono::ChFrame<double> *FrTidal::GetTidalFrame() const {
    return m_tidalFrame.get();
  }

  void FrTidal::Initialize() {
    event_logger::info("Tidal", "", "Tidal model initialization");
    event_logger::flush();
  }

  void FrTidal::StepFinalize() {

  }

  void FrTidal::SetNoTidal() { m_mode = NO_TIDAL; }


}  // end namespace frydom
