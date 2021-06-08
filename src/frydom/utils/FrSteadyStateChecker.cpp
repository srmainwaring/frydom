//
// Created by camille on 08/06/2021.
//

#include "FrSteadyStateChecker.h"

namespace frydom {

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