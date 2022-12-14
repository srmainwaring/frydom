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


#include <frydom/core/math/functions/ramp/FrCosRampFunction.h>
#include "FrEnvironment.h"

#include "frydom/core/math/functions/ramp/FrLinearRampFunction.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "time/FrTimeServices.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "ocean/freeSurface/FrFreeSurface.h"
#include "ocean/seabed/FrSeabed.h"
#include "ocean/current/FrCurrent.h"
#include "frydom/environment/atmosphere/FrAtmosphere.h"
#include "atmosphere/wind/FrWind.h"
#include "frydom/environment/geographicServices/FrGeographicServices.h"
#include "frydom/logging/FrEventLogger.h"


namespace frydom {

  FrEnvironment::FrEnvironment(FrOffshoreSystem *system) {

    m_system = system;

    m_geographicServices = std::make_unique<FrGeographicServices>();
    m_timeServices = std::make_unique<FrTimeServices>();
    m_ocean = std::make_unique<FrOcean>(this);
    m_atmosphere = std::make_unique<FrAtmosphere>(this);

    m_timeRamp = std::make_unique<FrCosRampFunction>();
    m_timeRamp->SetActive(false);
    m_timeRamp->SetByTwoPoints(0., 0., 10., 1.);

//        if (not(m_infinite_depth)) m_seabed->SetEnvironment(this); // TODO : voir a porter ca dans seabed...

    SetGravityAcceleration(9.81);
  }

  FrEnvironment::~FrEnvironment() = default;

  FrOffshoreSystem *FrEnvironment::GetSystem() { return m_system; }

  double FrEnvironment::GetTime() const { return m_system->GetTime(); } // TODO : voir a gerer l'UTC etc...


  double FrEnvironment::GetGravityAcceleration() const {
    return m_system->GetGravityAcceleration();
  }

  void FrEnvironment::SetGravityAcceleration(double gravityAcceleration) {
    m_system->SetGravityAcceleration(gravityAcceleration);
  }


  FrOcean *FrEnvironment::GetOcean() const { return m_ocean.get(); }

  FrAtmosphere *FrEnvironment::GetAtmosphere() const { return m_atmosphere.get(); }

  Velocity FrEnvironment::GetRelativeVelocityInFrame(const FrFrame &frame, const Velocity &worldVel,
                                                     FLUID_TYPE ft, FRAME_CONVENTION fc) {
    switch (ft) {
      case WATER:
        return m_ocean->GetCurrent()->GetFluxRelativeVelocityInFrame(frame, worldVel, fc);
      case AIR:
        return m_atmosphere->GetWind()->GetFluxRelativeVelocityInFrame(frame, worldVel, fc);
      default:
        throw FrException("Fluid is not known...");
    }
  }

  double FrEnvironment::GetFluidDensity(FLUID_TYPE ft) const {
    switch (ft) {
      case AIR:
        return m_atmosphere->GetDensity();
      case WATER:
        return m_ocean->GetDensity();
      default:
        throw FrException("Fluid is not known...");
    }
  }

  double FrEnvironment::GetFluidDensity(const Position &worldPos, FRAME_CONVENTION fc, bool waveDeformation) {
    // TODO: permettre un jour d'avoir des gradients de densite de l'eau...
    auto fluid_type = GetFluidTypeAtPointInWorld(worldPos, fc, waveDeformation);
    return GetFluidDensity(fluid_type);
  }

  FLUID_TYPE FrEnvironment::GetFluidTypeAtPointInWorld(const frydom::Position &worldPos,
                                                       frydom::FRAME_CONVENTION fc,
                                                       bool waveDeformation) {
    double waveElevation = 0.;
    if (waveDeformation) {
      waveElevation = m_ocean->GetFreeSurface()->GetElevation(worldPos.x(), worldPos.y(), fc);
    }

    if (IsNED(fc)) {
      if (worldPos.z() < waveElevation + DBL_EPSILON) {
        return FLUID_TYPE::AIR;
      } else {
        return FLUID_TYPE::WATER;
      }
    } else {
      if (worldPos.z() > waveElevation - DBL_EPSILON) {
        return FLUID_TYPE::AIR;
      } else {
        return FLUID_TYPE::WATER;
      }
    }
  }

  FrGeographicServices *FrEnvironment::GetGeographicServices() const {
    return m_geographicServices.get();
  }

  int FrEnvironment::GetYear() const {
    /// Get the UTC time to obtain the year
    auto lt = GetTimeServices()->GetUTCTime();
    date::year_month_day ymd{date::floor<date::days>(lt)};
    return int(ymd.year());
  }

  void FrEnvironment::ShowFreeSurface(bool show) {
    GetOcean()->GetFreeSurface()->Show(show);
  }

  void FrEnvironment::ShowSeabed(bool show) {
    GetOcean()->ShowSeabed(show);
  }

  FrTimeServices *FrEnvironment::GetTimeServices() const { return m_timeServices.get(); }

  void FrEnvironment::Update(double time) {
//        m_timeRamp->Update(time);
    m_ocean->Update(time);
    m_atmosphere->Update(time);
    m_timeServices->Update(time);
  }

  void FrEnvironment::Initialize() {

    event_logger::info("Environment", "", "BEGIN Environment initialization");
    event_logger::flush();

    m_timeRamp->Initialize();

    m_ocean->Initialize();
    m_atmosphere->Initialize();
    m_timeServices->Initialize();

    event_logger::info("Environment", "", "END Environment initialization");
    event_logger::flush();

  }

  void FrEnvironment::StepFinalize() {
    m_timeRamp->StepFinalize();
    m_ocean->StepFinalize();
    m_atmosphere->StepFinalize();
  }

  FrCosRampFunction *FrEnvironment::GetTimeRamp() const {
    return m_timeRamp.get();
  }


}  // end namespace frydom
