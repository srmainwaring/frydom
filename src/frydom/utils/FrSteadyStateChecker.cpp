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


#include "FrSteadyStateChecker.h"

namespace frydom {

  FrRecordingFieldBase::~FrRecordingFieldBase() {
  }

  FrRecordingFieldBase::FrRecordingFieldBase(double time_length, double time_step)
  : m_recorder(time_length, time_step),
    m_recorder_mean(time_length, time_step)
  {
    m_recorder.Initialize();
    m_recorder_mean.Initialize();
  }

  double FrRecordingFieldBase::GetStandardDeviation() {
    return m_recorder_mean.GetStandardDeviation();
  }

  double FrRecordingFieldBase::GetMean() {
    return m_recorder_mean.GetMean();
  }


} // end namespace frydom