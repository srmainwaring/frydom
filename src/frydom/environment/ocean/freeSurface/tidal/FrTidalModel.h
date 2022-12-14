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


#ifndef FRYDOM_FRTIDALMODEL_H
#define FRYDOM_FRTIDALMODEL_H

#include <memory>

#include "MathUtils/LookupTable1D.h"

#include "frydom/core/common/FrObject.h"
#include "frydom/core/common/FrConvention.h"

// TODO: La hauteur de marée (+ sonde a recuperer de seabed) doivent etre retranscrite sur le corps embarque dans la
// surface libre.

namespace chrono {
  template<class Real>
  class ChFrame;
}

namespace frydom {

  // FIXME: utiliser https://github.com/HowardHinnant/date comme sous module !!!

//    // TODO: utiliser boost ou une autre lib speciale pour la gestion des geoides
  /**
  * \class FrUTCTime
  * \brief Class for dealing with UTC time.
  */
  class FrUTCTime {
   private:
    unsigned int m_hours = 0;
    unsigned int m_minutes = 0;
    unsigned int m_seconds = 0;

    int m_local_correction = 0;

    void Check();

   public:
    FrUTCTime() : m_hours(0), m_minutes(0), m_seconds(0) {}

    FrUTCTime(unsigned int hours, unsigned int minutes, unsigned int seconds);

    FrUTCTime(unsigned int hours, unsigned int minutes);

    unsigned int GetH() const { return m_hours; }

    unsigned int GetM() const { return m_minutes; }

    unsigned int GetS() const { return m_seconds; }

    double GetHours() const;

    double GetMinutes() const;

    double GetSeconds() const;

    // TODO: ajouter operateurs de comparaison !!

  };

  // Forward declaration
  class FrFreeSurface;

  /**
   * \class FrTidal
   * \brief Class for defining tidals.
   */
  class FrTidal : public FrObject {

   public:
    enum TIDAL_LEVEL {
      LOW,
      HIGH
    };

    enum TIDAL_MODE {
      NO_TIDAL,
      TWELFTH_RULE
    };

   public:

    ~FrTidal() = default;

    explicit FrTidal(FrFreeSurface *freeSurface);

    FrTidal(FrFreeSurface *freeSurface, FrUTCTime t1, double h1, TIDAL_LEVEL level1, FrUTCTime t2,
            double h2, TIDAL_LEVEL level2);

    void SetNoTidal();

    void Update(double time);

    double GetHeight(FRAME_CONVENTION fc) const;

    const chrono::ChFrame<double> *GetTidalFrame() const;

    void Initialize() override;

    void StepFinalize() override;

   private:
    void BuildTable();

    void BuildTwelfthRuleTable();

   private:

    FrFreeSurface *m_freeSurface;

    std::unique_ptr<chrono::ChFrame<double>> m_tidalFrame;  // FIXME : utiliser un FrFrame !!!

    double m_time = 0.;

    TIDAL_MODE m_mode = NO_TIDAL;

    TIDAL_LEVEL m_level1;
    FrUTCTime m_t1;
    double m_h1 = 0.;

    TIDAL_LEVEL m_level2;
    FrUTCTime m_t2;
    double m_h2;

    mathutils::LookupTable1D<double, double> m_tidal_table;

  };

}  // end namespace frydom


#endif //FRYDOM_FRTIDALMODEL_H
