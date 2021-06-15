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


#ifndef FRYDOM_FRSTEADYSTATECHECKER_H
#define FRYDOM_FRSTEADYSTATECHECKER_H

#include <iostream>
#include "frydom/utils/FrRecorder.h"

namespace frydom {

  class FrRecordingFieldBase {

   public:

    FrRecordingFieldBase(double time_length, double time_step);

    virtual void Record(double time) = 0;

    double GetStandardDeviation();

    double GetMean();

   protected:

    FrTimeRecorder<double> m_recorder;
    FrTimeRecorder<double> m_recorder_mean;

  };

  template<class T>
  class FrRecordingField : public FrRecordingFieldBase {

   public:

    FrRecordingField(T *data, double time_length, double time_step)
        : FrRecordingFieldBase(time_length, time_step),
          m_data([data]() { return *data; }) {}

    FrRecordingField(std::function<T()> data, double time_length, double time_step)
        : FrRecordingFieldBase(time_length, time_step),
          m_data([data]() { return data(); }) {}

    virtual void Record(double time) override {
      m_recorder.Record(time, m_data());
      m_recorder_mean.Record(time, m_recorder.GetMean());
    }

   private:

    std::function<T()> m_data;

  };


  class FrSteadyStateChecker {

   public:

    FrSteadyStateChecker(double time_length, double time_step, double starting_time = 0.)
        : m_time_length(time_length), m_time_step(time_step), m_starting_time(starting_time) {}

    void Record(double time) {
      for (auto &field: m_fields) {
        field->Record(time);
      }
      m_is_active = time > m_starting_time;
    }

    bool IsConverged() {

      if (m_is_active) {

        auto is_steady = true;
        int i = 0;

        while (is_steady and i < m_fields.size()) {

          auto stdev = m_fields[i]->GetStandardDeviation();
          auto denom = std::max(m_fields[i]->GetMean(), 1.);

          is_steady = stdev / denom < m_max_deviations[i];
          std::cout << "** debug : stdev = " << stdev << " ; denom = " << denom << std::endl;
          std::cout << "** debug : stdev/denom = " << stdev/denom << std::endl;
          ++i;
        }
        return is_steady;
      }

      return false;
    }

    template<class T>
    void AddField(T *val, double max_deviation) {
      m_fields.emplace_back(std::make_unique<FrRecordingField<T>>(val, m_time_length, m_time_step));
      m_max_deviations.emplace_back(max_deviation);
    }

    template<typename T>
    void AddField(std::function<T()> func, double max_deviation) {
      m_fields.emplace_back(std::make_unique<FrRecordingField<T>>(func, m_time_length, m_time_step));
      m_max_deviations.emplace_back(max_deviation);
    }

   private:

    typedef std::vector<std::unique_ptr<FrRecordingFieldBase>> VectorType;
    VectorType m_fields;

    std::vector<double> m_max_deviations;

    double m_time_length;
    double m_time_step;
    double m_starting_time;
    bool m_is_active;

  };

} // end namespace frydom

#endif //FRYDOM_FRSTEADYSTATECHECKER_H